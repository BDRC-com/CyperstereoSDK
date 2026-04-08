#!/usr/bin/env python3

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception as exc:  # pragma: no cover
    CvBridge = None  # type: ignore
    _cv_bridge_import_error = exc


class _FrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._jpeg: Optional[bytes] = None
        self._seq: int = 0

    def update(self, jpeg: bytes) -> None:
        with self._cv:
            self._jpeg = jpeg
            self._seq += 1
            self._cv.notify_all()

    def get_latest(self) -> Tuple[Optional[bytes], int]:
        with self._lock:
            return self._jpeg, self._seq

    def wait_newer(
        self, last_seq: int, timeout_s: float
    ) -> Tuple[Optional[bytes], int]:
        with self._cv:
            if self._seq <= last_seq:
                self._cv.wait(timeout=timeout_s)
            return self._jpeg, self._seq


class _HttpHandler(BaseHTTPRequestHandler):
    server_version = "cyperstereo_mjpeg_streamer/0.1"

    def do_GET(self) -> None:  # noqa: N802
        frame_buffer: _FrameBuffer = self.server.frame_buffer  # type: ignore[attr-defined]
        max_fps: float = self.server.max_fps  # type: ignore[attr-defined]

        if self.path in ("/", "/index.html"):
            self._send_index()
            return

        if self.path in ("/snapshot.jpg", "/snapshot"):
            self._send_snapshot(frame_buffer)
            return

        if self.path in ("/stream", "/stream.mjpg", "/mjpeg"):
            self._send_stream(frame_buffer, max_fps)
            return

        self.send_response(404)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(b"Not Found\n")

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send_index(self) -> None:
        body = (
            "<html><head><meta charset='utf-8'></head>"
            "<body style='margin:0;background:#111'>"
            "<img src='/stream.mjpg' style='width:100vw;height:100vh;object-fit:contain' />"
            "</body></html>"
        ).encode("utf-8")

        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_snapshot(self, frame_buffer: _FrameBuffer) -> None:
        jpeg, _ = frame_buffer.get_latest()
        if not jpeg:
            self.send_response(503)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"No frame yet\n")
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_stream(self, frame_buffer: _FrameBuffer, max_fps: float) -> None:
        boundary = b"frame"
        self.send_response(200)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={boundary.decode()}"
        )
        self.end_headers()

        last_seq = -1
        min_period = 0.0 if max_fps <= 0.0 else (1.0 / max_fps)
        last_send_t = 0.0

        try:
            while True:
                jpeg, seq = frame_buffer.wait_newer(last_seq, timeout_s=1.0)
                if not jpeg:
                    continue

                now = time.time()
                dt = now - last_send_t
                if min_period > 0.0 and dt < min_period:
                    time.sleep(min_period - dt)

                part = (
                    b"--" + boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.write(part)
                self.wfile.flush()

                last_seq = seq
                last_send_t = time.time()
        except (BrokenPipeError, ConnectionResetError):
            return


class MjpegStreamer(Node):
    def __init__(self) -> None:
        super().__init__("mjpeg_streamer")

        self.declare_parameter("enabled", False)
        self.declare_parameter("image_topic", "/cam0/image_raw")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("jpeg_quality", 80)

        self.enabled = bool(self.get_parameter("enabled").value)
        self._httpd = None

        if not self.enabled:
            self.get_logger().info("MJPEG streamer disabled by parameter.")
            return

        if CvBridge is None:
            raise RuntimeError(
                "cv_bridge import failed; please install ROS 2 cv_bridge. "
                f"Original error: {_cv_bridge_import_error}"
            )

        self._bridge = CvBridge()
        self._frame_buffer = _FrameBuffer()

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self._sub = self.create_subscription(Image, image_topic, self._on_image, 10)

        bind_address = (
            self.get_parameter("bind_address").get_parameter_value().string_value
        )
        port = int(self.get_parameter("port").value)
        max_fps = float(self.get_parameter("max_fps").value)

        self._httpd = ThreadingHTTPServer((bind_address, port), _HttpHandler)
        self._httpd.frame_buffer = self._frame_buffer  # type: ignore[attr-defined]
        self._httpd.max_fps = max_fps  # type: ignore[attr-defined]

        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True
        )
        self._http_thread.start()

        self.get_logger().info(
            f"MJPEG stream: http://{bind_address}:{port}/ (topic: {image_topic}, max_fps: {max_fps})"
        )

    def destroy_node(self) -> bool:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()

    def _on_image(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {exc}")
            return

        try:
            import cv2

            ok, encoded = cv2.imencode(
                ".jpg",
                cv_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)],
            )
            if not ok:
                return
            self._frame_buffer.update(encoded.tobytes())
        except Exception as exc:
            self.get_logger().warn(f"JPEG encode failed: {exc}")


def main() -> None:
    rclpy.init()
    node = MjpegStreamer()
    if not node.enabled:
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()