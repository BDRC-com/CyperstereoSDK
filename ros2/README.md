# ROS2 Build and Run

```bash
cd ~/CyperstereoSDK/ros2
source /opt/ros/humble/setup.sh
colcon build --symlink-install
source install/setup.sh
ros2 run cyperstereo_ros2_bridge cyperstereo_ros2_bridge \
	--ros-args \
	--params-file src/cyperstereo_ros2_bridge/config/capture_image_imu.yaml
```

或直接通过 launch：

```bash
ros2 launch cyperstereo_ros2_bridge cyperstereo_ros2_bridge.launch.py
```

# Parameter File

默认参数文件位于 [src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/config/capture_image_imu.yaml](src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/config/capture_image_imu.yaml)。

当前节点名为 `cyperstereo_ros2_bridge`。

可配置项包括：

- 设备采集参数：分辨率、帧率、像素格式、stream index
- 采集参数：重连间隔、队列长度、队列满时丢弃策略
- 图像发布参数：发布频率、左右图像 topic、frame_id、encoding
- IMU 发布参数：发布频率、topic、frame_id、重力缩放
- 调试开关：左右图像时间戳日志、IMU 时间戳日志
- QoS：每个 publisher 独立的 history、depth、reliability、durability

当前默认配置下：

- 图像采集频率由 `device.fps` 控制，默认 50 Hz
- 图像发布频率由 `image.publish_hz` 控制，默认 20 Hz，可直接改成 10、25 或 50
- IMU 发布频率由 `imu.publish_hz` 控制，默认 200 Hz，可直接改成 100
- 图像与 IMU 的 `header.stamp` 都来自设备 SDK 提供的硬件时间戳，桥节点只负责把秒单位时间戳转成 ROS 时间，不会改写成节点当前系统时间

如果把 `image.publish_hz` 或 `imu.publish_hz` 设为小于等于 0，则表示不做发布限频，按采集到的数据全量发布。

注意：当前默认仍是 50 Hz 图像采集再限频到 20 Hz 发布，因此图像时间戳仍然是被选中那一帧的原始硬件时间戳。若你需要严格的原生 20 Hz 采集节奏，而不是 50 Hz 输入上的限频发布，需要同时调整 `device.fps`，前提是设备模式本身支持该帧率。

Launch 文件位于 [src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/launch/cyperstereo_ros2_bridge.launch.py](src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/launch/cyperstereo_ros2_bridge.launch.py)。


