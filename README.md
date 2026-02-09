# detect 功能包

本 `detect` 包实现一个基于 ROS2 的数字识别节点，用于订阅摄像头图像并发布识别结果（数字类别与置信度）。
此 README 按照作业二的要求整理了使用方法、结构与调试说明。

**主要功能**
- 订阅相机图像话题（本仓库中节点订阅的默认话题为 `/camera1/image_raw`）
- 识别图片中的数字卡片（0-9）
- 发布识别结果：`/digit_class`（`std_msgs/Int32`），以及可选的置信度：`/digit_score`（`std_msgs/Float32`）

如果缺少依赖，在系统上安装后重新编译工作空间。

**构建（在工作空间根目录下执行）**
```bash
cd ~/桌面/ros2-camera-process-node-homework
colcon build
source install/setup.bash
```

**一键启动（launch）**
- 启动 usb_cam + detect（推荐在已 `source` 的终端下运行）：
```bash
ros2 launch detect detect.launch.py
```

说明：如果你的相机话题是 `/image_raw` 或其它名称，请在 `launch/detect.launch.py` 或 `detect_node` 中调整订阅话题为对应名称。

**查看/调试**
- 查看识别结果：
```bash
ros2 topic echo /digit_class
ros2 topic echo /digit_score
```

**如何准备数字卡片与模板**
- 将你制作的数字图片放入 `src/detect/templates/`，文件名示例 `0white.png` 到 `9white.png`（detect_node 代码中按此格式加载）
- 如果改变命名或路径，请同时更新 `detect_node.cpp` 中加载模板的路径

**故障排查**
- `colcon build` 找不到包：确保你的工作空间根目录没有多余的 `CMakeLists.txt`（否则 colcon 会把根目录当包），并且 `src/detect/package.xml` 存在。
- Pylance 报 `Import 'camera_config' could not be resolved`：这是编辑器静态检查问题，实际运行使用 `ament_index_python.packages.get_package_share_directory` 获取 `usb_cam` 目录即可。
- 摄像头无法打开：确认摄像头设备节点权限，并可用 `v4l2-ctl --list-devices` 检查设备。
