#!/usr/bin/env python3

import threading
import time
from dataclasses import dataclass
import sys
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception as exc:  # pragma: no cover
    CvBridge = None  # type: ignore
    _cv_bridge_import_error = exc

try:
    import gi  # type: ignore[import-not-found]

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")

    from gi.repository import GLib, Gst, GstRtspServer  # type: ignore[import-not-found]
except Exception as exc:  # pragma: no cover
    gi = None  # type: ignore
    GLib = None  # type: ignore
    Gst = None  # type: ignore
    GstRtspServer = None  # type: ignore
    _gi_import_error = exc


@dataclass(frozen=True)
class _Frame:
    data: bytes
    width: int
    height: int


class _FrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._frame: Optional[_Frame] = None
        self._seq: int = 0

    def update(self, frame: _Frame) -> None:
        with self._cv:
            self._frame = frame
            self._seq += 1
            self._cv.notify_all()

    def wait_newer(
        self, last_seq: int, timeout_s: float
    ) -> Tuple[Optional[_Frame], int]:
        with self._cv:
            if self._seq <= last_seq:
                self._cv.wait(timeout=timeout_s)
            return self._frame, self._seq


def _make_sensor_factory_class():
    if Gst is None or GstRtspServer is None:
        detail = str(_gi_import_error) if "_gi_import_error" in globals() else "unknown error"
        raise RuntimeError(
            "GStreamer RTSP server bindings are unavailable. "
            "Install python3-gi and gir1.2-gst-rtsp-server-1.0. "
            f"Original error: {detail}"
        )

    class _SensorFactory(GstRtspServer.RTSPMediaFactory):
        def __init__(
            self,
            frame_buffer: _FrameBuffer,
            width: int,
            height: int,
            fps: int,
            bitrate_kbps: int,
            key_int_max: int,
            encoder: str,
        ) -> None:
            super().__init__()
            self._frame_buffer = frame_buffer
            self._width = width
            self._height = height
            self._fps = max(1, fps)
            self._bitrate_kbps = max(1, bitrate_kbps)
            self._key_int_max = max(1, key_int_max)
            self._encoder = (encoder or "x264").strip().lower()

            self._duration = int(Gst.SECOND / self._fps)
            self._frame_count = 0
            self._last_seq = -1
            self._last_push_t = 0.0
            self._launch = self._build_launch()

            self.set_shared(True)

        def _build_launch(self) -> str:
            common_prefix = (
                "appsrc name=src is-live=true block=true format=time do-timestamp=true "
                f"caps=video/x-raw,format=BGR,width={self._width},height={self._height},framerate={self._fps}/1 "
                "! queue leaky=downstream max-size-buffers=1 "
                "! videoconvert "
            )

            if self._encoder in ("x264", "x264enc", "sw"):
                return (
                    common_prefix
                    + "! video/x-raw,format=I420 "
                    + f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={self._bitrate_kbps} key-int-max={self._key_int_max} "
                    + "! rtph264pay name=pay0 pt=96 config-interval=1"
                )

            if self._encoder in ("nvv4l2", "nvv4l2h264enc", "jetson", "nv"):
                bitrate_bps = int(self._bitrate_kbps) * 1000
                return (
                    common_prefix
                    + "! video/x-raw,format=NV12 "
                    + f"! nvv4l2h264enc bitrate={bitrate_bps} iframeinterval={self._key_int_max} insert-sps-pps=true "
                    + "! h264parse "
                    + "! rtph264pay name=pay0 pt=96 config-interval=1"
                )

            if self._encoder in ("vaapi", "vaapih264enc"):
                return (
                    common_prefix
                    + "! video/x-raw,format=I420 "
                    + f"! vaapih264enc bitrate={self._bitrate_kbps} keyframe-period={self._key_int_max} "
                    + "! h264parse "
                    + "! rtph264pay name=pay0 pt=96 config-interval=1"
                )

            return (
                common_prefix
                + "! video/x-raw,format=I420 "
                + f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={self._bitrate_kbps} key-int-max={self._key_int_max} "
                + "! rtph264pay name=pay0 pt=96 config-interval=1"
            )

        def do_create_element(self, url):  # noqa: N802
            return Gst.parse_launch(self._launch)

        def do_configure(self, rtsp_media):  # noqa: N802
            pipeline = rtsp_media.get_element()
            appsrc = pipeline.get_by_name("src")
            appsrc.connect("need-data", self._on_need_data)

        def _on_need_data(self, src, length) -> None:
            now = time.time()
            dt = now - self._last_push_t
            min_period = 1.0 / float(self._fps)
            if dt < min_period:
                time.sleep(min_period - dt)

            frame, seq = self._frame_buffer.wait_newer(self._last_seq, timeout_s=1.0)
            if frame is None:
                return

            buf = Gst.Buffer.new_allocate(None, len(frame.data), None)
            buf.fill(0, frame.data)
            buf.duration = self._duration
            buf.pts = self._frame_count * self._duration
            buf.dts = buf.pts

            self._frame_count += 1
            self._last_seq = seq
            self._last_push_t = time.time()

            src.emit("push-buffer", buf)

    return _SensorFactory


class RtspStreamer(Node):
    def __init__(self) -> None:
        super().__init__("rtsp_streamer")

        self.declare_parameter("enabled", False)
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8554)
        self.declare_parameter("mount_point", "/cyperstereo")
        self.declare_parameter("fps", 15)
        self.declare_parameter("bitrate_kbps", 2000)
        self.declare_parameter("key_int_max", 15)
        self.declare_parameter("encoder", "x264")

        self.enabled = bool(self.get_parameter("enabled").value)

        if not self.enabled:
            self.get_logger().info("RTSP streamer disabled by parameter.")
            return

        if CvBridge is None:
            raise RuntimeError(
                "cv_bridge import failed; please install ROS 2 cv_bridge. "
                f"Original error: {_cv_bridge_import_error}"
            )

        if gi is None or Gst is None or GstRtspServer is None or GLib is None:
            raise RuntimeError(
                "GStreamer (gi) import failed; please install python3-gi and GStreamer RTSP server bindings. "
                f"Original error: {_gi_import_error}"
            )

        self._bridge = CvBridge()
        self._frame_buffer = _FrameBuffer()

        self._server_thread: Optional[threading.Thread] = None
        self._loop: Optional[Any] = None
        self._server_started = False
        self._first_dims: Optional[Tuple[int, int]] = None

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._sub = self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"RTSP streamer waiting for first frame on topic: {image_topic}"
        )

    def destroy_node(self) -> bool:
        try:
            if self._loop is not None:
                self._loop.quit()
        except Exception:
            pass
        return super().destroy_node()

    def _start_server_if_needed(self, width: int, height: int) -> None:
        if self._server_started:
            return

        bind_address = (
            self.get_parameter("bind_address").get_parameter_value().string_value
        )
        port = int(self.get_parameter("port").value)
        mount_point = (
            self.get_parameter("mount_point").get_parameter_value().string_value
        )
        fps = int(self.get_parameter("fps").value)
        bitrate_kbps = int(self.get_parameter("bitrate_kbps").value)
        key_int_max = int(self.get_parameter("key_int_max").value)
        encoder = self.get_parameter("encoder").get_parameter_value().string_value

        if not mount_point.startswith("/"):
            mount_point = "/" + mount_point

        def _run() -> None:
            Gst.init(None)

            server = GstRtspServer.RTSPServer()
            server.set_address(bind_address)
            server.set_service(str(port))

            mounts = server.get_mount_points()
            sensor_factory_class = _make_sensor_factory_class()
            factory = sensor_factory_class(
                frame_buffer=self._frame_buffer,
                width=width,
                height=height,
                fps=fps,
                bitrate_kbps=bitrate_kbps,
                key_int_max=key_int_max,
                encoder=encoder,
            )
            mounts.add_factory(mount_point, factory)
            server.attach(None)

            self.get_logger().info(
                f"RTSP stream ready: rtsp://{bind_address}:{port}{mount_point} "
                f"(size: {width}x{height}, fps: {fps}, bitrate_kbps: {bitrate_kbps}, encoder: {encoder})"
            )

            self._loop = GLib.MainLoop()
            self._loop.run()

        self._server_thread = threading.Thread(target=_run, daemon=True)
        self._server_thread.start()
        self._server_started = True

    def _on_image(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {exc}")
            return

        height, width = cv_img.shape[:2]

        if self._first_dims is None:
            self._first_dims = (width, height)
            self._start_server_if_needed(width=width, height=height)
        elif (width, height) != self._first_dims:
            width, height = self._first_dims

        frame = _Frame(data=cv_img.tobytes(), width=width, height=height)
        self._frame_buffer.update(frame)


def main() -> None:
    rclpy.init()
    node = None
    node = RtspStreamer()
    try:
        if not node.enabled:
            return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger("rtsp_streamer").error(str(exc))
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()