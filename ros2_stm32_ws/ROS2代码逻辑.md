# ROS2 代码逻辑

本文梳理 `ros2_stm32_ws` 的 ROS2 上位机逻辑，并解释 `tf2_echo map odom` 报
`Invalid frame ID "map"` 时应该从哪里排查。

## 1. 工作区角色

这个工作区负责把 STM32 底盘、雷达、IMU、SLAM 和 Nav2 串成一套机器人系统：

| 功能包 | 主要职责 |
| --- | --- |
| `stm32_nav_comm` | 通过串口和 STM32 通信；订阅 `/cmd_vel`；发布 `/odom`、`odom -> base_link`、`/stm32/control_mode`、`/stm32/nav_state` |
| `stm32_nav_bringup` | 统一启动传感器、建图、导航、RViz；保存配置文件 |
| `sllidar_ros2` | 启动思岚雷达，发布 `/scan` |
| `yesense_std_ros2` | 启动 IMU，发布 IMU 和姿态相关话题 |
| `imu_tf_broadcaster` | 可选：根据 IMU 姿态发布 IMU TF |
| `serial_ros2` | C++ 串口库，供 `stm32_nav_comm` 链接 |

## 2. TF 坐标树

正常系统至少需要这条 TF 链：

```text
map -> odom -> base_link -> laser
                     |
                     +-> gyro_link
```

每一段由不同节点负责：

| TF | 发布者 | 说明 |
| --- | --- | --- |
| `map -> odom` | 建图模式：`slam_toolbox`；导航模式：`amcl` | 全局地图到局部里程计的校正关系 |
| `odom -> base_link` | `stm32_comm_node`，或 `laser_odometry.py` | 机器人相对启动点的连续里程计 |
| `base_link -> laser` | `tf2_ros static_transform_publisher` | 雷达安装位姿 |
| `base_link -> gyro_link` | `tf2_ros static_transform_publisher` | IMU 安装位姿 |

`tf2_echo map odom` 报 `Invalid frame ID "map"`，含义是 TF 树里还没有任何节点发布
`map` frame。它不是 `tf2_echo` 命令错误，而是 `map -> odom` 这一段没有出现。

## 3. 三种启动方式

### 3.1 传感器单测

```bash
ros2 launch stm32_nav_bringup test_sensors.launch.py
```

传感器单测不会启动 SLAM 或 AMCL，本来不会有真实 `map -> odom`。为了让 RViz 固定
坐标系和 `tf2_echo map odom` 能工作，`test_sensors.launch.py` 默认启用临时静态
`map -> odom`，同时默认不启动 STM32 底盘通信节点，避免单测传感器时轮子误动：

```text
publish_identity_map_odom:=true
enable_stm32:=false
```

这个静态 TF 只适合单测。正式建图和导航不要打开它，否则会和 `slam_toolbox` 或
`amcl` 抢同一条 TF。

如果确实要在单测时同时查看 STM32 里程计，可显式打开：

```bash
ros2 launch stm32_nav_bringup test_sensors.launch.py enable_stm32:=true
```

### 3.2 建图

```bash
ros2 launch stm32_nav_bringup slam_launch.py
```

建图默认是手推安全模式：不启动 STM32 底盘通信，使用 `laser_odometry.py`
发布 `/odom` 和 `odom -> base_link`，避免下位机接管电机导致轮子乱转。

默认启动内容：

```text
sensors.launch.py
  -> yesense_pub
  -> sllidar_node
  -> base_link 静态 TF
laser_odometry.py
slam_toolbox
map_saver_server
rviz2
```

如果确实要使用 STM32 轮速里程计建图，必须显式打开：

```bash
ros2 launch stm32_nav_bringup slam_launch.py \
  enable_stm32:=true use_laser_odom:=false
```

注意：当前实测 STM32 会出现 `control_mode` 跳回 `0` 且轮速非零的问题，所以在下位机
模式仲裁修好前，不建议用这条命令手推建图。

此时 `map -> odom` 应该由 `slam_toolbox` 发布。若没有出现，重点检查：

```bash
ros2 node list | grep slam
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

只有 `/scan`、`/odom` 和机器人本体 TF 都正常，`slam_toolbox` 才能持续发布
`map -> odom`。

### 3.3 导航

```bash
ros2 launch stm32_nav_bringup nav_launch.py \
  map:=/path/to/map.yaml
```

导航模式会启动 Nav2 和 AMCL。`map -> odom` 由 AMCL 发布，前提是：

1. `map:=...` 传入有效地图 yaml。
2. `/scan`、`/odom`、`odom -> base_link`、`base_link -> laser` 都存在。
3. AMCL 有初始位姿。

当前配置已在 `nav2_params.yaml` 中设置：

```yaml
set_initial_pose: true
initial_pose:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
```

这会让 AMCL 启动后先用地图原点作为初始位姿发布 `map -> odom`。如果机器人实际不在
地图原点，启动后应在 RViz 用 `2D Pose Estimate` 重新发 `/initialpose` 校正。

## 4. STM32 通信节点逻辑

核心文件：

```text
src/stm32_nav_comm/include/stm32_nav_comm/stm32_comm_node.h
src/stm32_nav_comm/src/stm32_comm_node.cpp
src/stm32_nav_bringup/config/stm32_comm.yaml
```

串口协议：

```text
0xAA | CMD | LEN | DATA | XOR | 0x55
```

关键命令：

| 命令 | 方向 | 内容 |
| --- | --- | --- |
| `CMD_VEL = 0x01` | ROS2 -> STM32 | `vx, vy, wz` 三个 float |
| `CMD_ODOM = 0x04` | STM32 -> ROS2 | `tick_ms, control_mode, nav_state, omega_a, omega_b, omega_d` |
| `CMD_MODE = 0x05` | ROS2 -> STM32 | 手动模式/ROS2 模式 |

主流程：

1. 构造函数读取参数，打开串口，先连续发送 `MODE_ROS2 + 0速度`，再创建 `/cmd_vel` 订阅、`/odom` 发布和 TF 广播器。
2. `cmdVelCallback()` 接收 ROS 坐标系速度：`x` 前、`y` 左、`z` 逆时针。
3. 回调内先补发 `MODE_ROS2`，再转换成 STM32 坐标系：`x` 右、`y` 前、`z` 逆时针，然后 `sendCmdVel()` 打包发送。
4. `pollSerial()` 每 5ms 检查 `/cmd_vel` 是否超时、STM32 是否回到非 ROS2 模式；需要时调用 `sendSafetyStop()` 补发 `MODE_ROS2 + 0速度`。
5. `pollSerial()` 读取串口，把字节放入 `rx_buffer_`。
6. `tryParseOneFrame()` 从缓存中找帧头、检查长度、帧尾和异或校验。
7. `handleFrame()` 按命令字分发，里程计帧进入 `handleOdomFrame()`。
8. `updateOdometryFromWheelSpeeds()` 将三轮角速度换算为 ROS 速度，并用中点法积分 `x/y/yaw`。
9. `publishOdometry()` 发布 `/odom`，`publishTransform()` 发布 `odom -> base_link`。

当前串口默认值已改成 `/dev/serial/by-id/...`，避免 `/dev/ttyUSB0` 和 `/dev/ttyUSB1`
在重插后互换：

| 设备 | 默认路径 |
| --- | --- |
| STM32 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| SLLidar | `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f69570a8c213ec118f16f4ef7a109228-if00-port0` |
| IMU | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C092620-if00` |

## 5. 激光里程计逻辑

核心文件：

```text
src/stm32_nav_bringup/scripts/laser_odometry.py
```

这个节点只在 `use_laser_odom:=true` 时启动，用作轮速里程计的替代调试方案。

主流程：

1. 订阅 `/scan`。
2. `scan_to_points()` 把激光极坐标转换成二维点云。
3. `match_scans()` 用 KDTree 粗搜索相邻两帧的 `dx/dy/dtheta`。
4. `scan_callback()` 把局部增量积分到 odom 坐标系。
5. 发布 `/odom`，并按参数决定是否发布 `odom -> base_link`。

注意：这个实现是轻量调试方案，不是完整 ICP/SLAM。环境特征少、速度快或激光噪声大时，
估计会漂移。当前脚本已加入静止门限：机器人不动时 `/odom` 话题仍会约 10Hz 发布，
但 pose 应保持不变；如果 pose 继续明显变化，需要调高 `stationary_score_margin` 或
改用更成熟的激光里程计/轮速里程计。脚本也加入了“纯旋转优先”判断：原地转向时，
只有带平移的匹配明显更好，才会积分平移；否则只更新 yaw，减少旋转时墙体被拖出重影。

## 6. 排查 `map` frame 不存在

优先按模式判断：

| 当前启动方式 | 正确处理 |
| --- | --- |
| 只测传感器 | 用 `test_sensors.launch.py`，或给 `sensors.launch.py` 显式传 `publish_identity_map_odom:=true` |
| 建图 | 确认 `slam_toolbox` 正在运行并能收到 `/scan`、`/odom` 和 TF |
| 导航 | 必须传入 `map:=地图.yaml`，并确认 AMCL 已收到初始位姿 |

常用命令：

```bash
ros2 topic list
ros2 node list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_tools view_frames
```

如果 `odom -> base_link` 有、`base_link -> laser` 有，但 `map -> odom` 没有，问题一般在
`slam_toolbox` 或 AMCL，而不是 STM32 串口节点。

## 7. 当前实测结论

本机最新测试中，STM32 串口已经能打开并收到 `/odom`，但反馈里 `control_mode` 会在
`0/1` 之间跳，且偶尔还有非零轮速：

```text
STM32 is not in ROS2 mode (mode=0); enforcing ROS2 mode and zero velocity
STM32 odom: ... mode=0 ... omega=[30.610 5.236 0.000]
STM32 odom: ... mode=1 ... omega=[0.000 0.000 0.000]
```

这说明 ROS2 上位机已经在补发 ROS2 模式和零速度，但下位机或手柄输入仍可能把底盘拉回
手动模式。正式建图前应先让下面两个命令稳定：

```bash
ros2 topic echo /stm32/control_mode
ros2 topic hz /odom
```

`/stm32/control_mode` 应长期为 `1`，`/odom` 应持续发布；这时再看：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

如果 `control_mode` 还在跳回 `0`，需要检查下位机模式仲裁或手柄 `START/模式切换` 输入；
仅靠 `slam_toolbox` 不能修复这个底盘控制状态。

为了先验证 SLAM 本身，可临时绕开 STM32 轮速里程计，让激光里程计发布
`odom -> base_link`，同时不启动 STM32 底盘通信：

```bash
ros2 launch stm32_nav_bringup slam_launch.py \
  enable_stm32:=false use_laser_odom:=true
```

这个模式适合排查 `map -> odom` 和雷达/TF 链路，不适合作为最终高精度导航方案。
如果只想验证 `slam_toolbox` 能不能发布 `map -> odom`，并且机器人保持静止，也可以用：

```bash
ros2 launch stm32_nav_bringup slam_launch.py \
  enable_stm32:=false use_laser_odom:=false
```

此时 `sensors.launch.py` 会补一个临时静态 `odom -> base_link`，方便确认雷达和
`slam_toolbox` 本身是否正常。
