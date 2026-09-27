# MID360S 本地实施审计（C0，2026-09-27）

## 版本和环境

- 本地项目 `drone`：`80a2add3b864600716f0edd0aaf088cb7886c819`，审计前主工作树干净。
- `ros2_ws/src/livox_ros_driver2`：`21445540f0d100dc86a7e6df312dd70bbdb4afdf`，有项目配置和 launch 覆盖；本地来源锁定于 `ros2_ws/setup_localization_sources.sh`。
- `ros2_ws/src/FAST_LIO_ROS2`：`2fffc570a25d0df172720bac034fbdb6a13d2162`，工作树应用了 `ros2_ws/patches/fast_lio_highrate.patch`，另有 bench 配置覆盖。两个 `src` 仓库不由项目根仓库跟踪；可重现修改必须写入项目跟踪的补丁/配置。
- 飞控电脑的现有记录（`FC_AUTOSTART_INVENTORY.md`、`ros2_ws/README.md`）为 Intel N97、Ubuntu 22.04、ROS 2 Humble。本地审计机是 Windows；**本次没有重新登录飞控电脑核验 OS、架构、包版本或编译**。
- 附带 `audit_repo.py` 在当前 Windows Git 环境执行失败：`git submodule status --recursive` 所需的 `basename`、`sed`、`git-sh-setup` 无法在其 shell 中解析。已改用只读 `git rev-parse`、`git ls-files`、`git status`、`rg` 与逐文件阅读。未把该脚本记为通过。

## 现有链路与接口

| 核验项 | 本地代码证据与结论 |
| --- | --- |
| 驱动与时间 | `ros2_ws/src/livox_ros_driver2/launch/msg_MID360s_launch.py` 选择 S 配置；`src/lddc.cpp:480` 直接以设备 `time_stamp` 构造 ROS IMU stamp。`src/comm/pub_handler.cpp:112` 从原始 IMU 包读六轴值。点云和 IMU 的共同 epoch、源时间对主机时间的年龄仍未核验。 |
| 话题 | `src/FAST_LIO_ROS2/src/laserMapping.cpp` 订阅 `/livox/lidar`、`/livox/imu`，发布 `/Odometry` 和 `/Odometry_highrate`。`python_sdk/FlightController/Components/RosNode.py:LioListenNode` 订阅高频话题。两个 odom 的 frame 均为 `camera_init`、`body`；`body` 在当前实现中实际指 IMU 原点，不是已转换的飞机机体原点。 |
| S 点处理 | `src/FAST_LIO_ROS2/src/preprocess.cpp:process(CustomMsg)` 总是进入 `avia_handler`；旧 tag 判据 `(tag & 0x30)`、`line < N_SCANS` 不符合计划的 S 原始点路径。`sync_packages()` 从**过滤后**末点和运行均值推断帧末，尚未保留原始最大 offset。 |
| 高频传播 | `src/FAST_LIO_ROS2/src/laserMapping.cpp:imu_cbk` 对相邻真实 IMU 调用 `PropagateHighRateState`，所以不是简单的 200 Hz 重发；但激光更新后在 `timer_callback()` 直接把 `state_highrate` 重置为延迟的 `state_point`，未重放锚点之后缓存的 IMU；也未传播完整协方差。当前高频状态**不符合**计划 C3。 |
| 激光更新质量 | `laserMapping.cpp` 每帧调用迭代滤波更新后即发布 odom。尚无绑定 correction sequence、有效匹配点数、残差、几何退化和 epoch 的质量消息；预测状态可被消费者当作有效定位。`publish_odometry()` 在填 covariance 前 publish，且协方差索引需要按本地状态顺序复核。 |
| 机体外参 | `python_sdk/FlightController/Components/LioPoseProvider.py` 从 `python_sdk/config/mid360s_mount.json` 读取 `T_I_B`，应用旋转杆臂位置和姿态。模板要求实测值；本地没有生产安装文件。未实现杆臂速度、重力竖直对齐、外参复核标志、质量授权或进程 epoch 检测。 |
| Navigation | `python_sdk/FlightController/Solutions/Navigation.py` 的控制位置为 cm、`current_yaw` 为顺时针正度数，`calibrate_basepoint()` 通过 provider 建立一次局部原点。`_navigation_task()` 周期性检查 `get_pose()`；失效时通过原 `fc.send_realtime_control_data` 路径发送水平零控制。该零控制不证明飞机能安全悬停。 |
| 串口与启动 | `python_sdk/server_ros.py` 构造唯一 `FC_Server` 并通过现有 `RosManager` 启动驱动和 LIO。使命启动前检查四个**话题名称**存在，但话题存在不代表有实时 IMU 消息或可靠激光校正。无第二个定位/串口发布者的结论仅限此启动路径；历史任务脚本仍有其他入口。 |
| 高度与坐标 | `LOCAL_TO_FIELD_COORDINATE_CONTRACT.md`：飞机任务局部坐标由基点校准，导航 yaw 顺时针；FleetBus REPORT 为 cm、逆时针 heading。Navigation 高度继续从飞控遥测获取；LIO 相对 z 不是离地高度。 |

## C0 回归与阻断项

- 在 Windows 本地运行 `python -m pytest -q python_sdk/testcode/test_lio_pose_provider.py python_sdk/testcode/test_navigation_lio_integration.py python_sdk/testcode/test_server_ros_lio_gate.py`：**8 passed**。均为纯逻辑/stub，无硬件连接。
- `ros2_ws/README.md` 记录的 2026-09-27 现场短测：点云约 9.98 Hz，但 `/livox/imu` 四秒内 **0 条消息**；FAST-LIO 未启动。该结果阻断 C1 实际完成条件，也阻断后续实机签收。需先用独占驱动、原始 SDK 数据、话题 QoS/频率及网络端口定位 IMU 缺失原因。
- 当前缺少真实 `T_I_B` 测量、正常 IMU 数据、rosbag、scan correction 统计与安全失效路径的现场验证。软件不得因 C0 静态审计或测试通过而放行导航/飞行。
