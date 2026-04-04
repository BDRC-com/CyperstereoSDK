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

- 设备采集参数：分辨率、像素格式、stream index
- 采集参数：重连间隔、队列长度、队列满时丢弃策略
- 图像发布参数：整数步长抽样、左右图像 topic、frame_id、encoding
- IMU 发布参数：整数步长抽样、topic、frame_id、重力缩放
- 调试开关：左右图像时间戳日志、IMU 时间戳日志
- QoS：每个 publisher 独立的 history、depth、reliability、durability

当前默认配置下：

- 设备采样请求固定为 60 Hz，不再通过参数暴露；根据当前设备和驱动实测，主机侧无法可靠切换到 25 或 50，实际输出频率仍应以实测结果为准
- 图像发布由 `image.publish_stride` 控制，表示每 `N` 帧发布 1 帧；例如 `1` 表示全发，`2` 表示 2 取 1，`3` 表示 3 取 1。默认值为 `2`，更偏向 VINS 场景下保留 IMU、适度降低图像负载
- IMU 发布由 `imu.publish_stride` 控制，表示每 `N` 个 IMU 样本发布 1 个；例如 `1` 表示全发，`2` 表示 2 取 1。默认值为 `1`
- 图像与 IMU 的 `header.stamp` 都来自设备 SDK 提供的硬件时间戳，桥节点只负责把秒单位时间戳转成 ROS 时间，不会改写成节点当前系统时间
- 图像和 IMU 的降频都使用固定整数步长抽样，也就是严格 `n 取 1`，从而避免使用 `Hz` 参数时对实际设备输入频率的歧义

节点启动时会额外打印提示：`60` 只是设备协商请求值，不代表真实输入帧率；真实输入请通过长跑 `analyze_capture_timing` 程序实测后，再决定 `publish_stride`。

如果 `publish_stride < 1`，节点会在启动时报错并拒绝运行。

Launch 文件位于 [src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/launch/cyperstereo_ros2_bridge.launch.py](src/CyperstereoSDK/ros2/src/cyperstereo_ros2_bridge/launch/cyperstereo_ros2_bridge.launch.py)。


