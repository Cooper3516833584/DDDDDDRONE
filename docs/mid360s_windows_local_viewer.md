# Windows 本机雷达三维轨迹测试

Windows 的 `livox` Conda 环境负责 Matplotlib/Tk 窗口，本机 WSL Ubuntu 22.04 / ROS2 Humble 运行 Livox 驱动和 FAST-LIO。查看器通过本地 WSL 标准输入输出订阅定位，不连接小电脑。

## 本机环境

- Conda Python：`C:\Users\TZDEZACR\miniconda3\envs\livox\python.exe`，Python 3.13。
- GUI 依赖：`matplotlib`；Tk 和 NumPy 已可用。
- 本机以太网：`192.168.1.50/24`；雷达：`192.168.1.194`。
- `%USERPROFILE%\.wslconfig` 使用 `networkingMode=mirrored`，`[experimental] hostAddressLoopback=true`。
- Windows 防火墙规则 `MID360S-Local-UDP` 和 Hyper-V 规则 `MID360S-WSL-UDP`，只允许来自雷达 IP 的 UDP 56000、56101、56201、56301、56401、56501。

WSL 配置修改需 `wsl --shutdown` 后生效，会关闭已有 WSL 会话。防火墙规则修改需管理员权限。

## 运行

驱动与 FAST-LIO 每项只运行一个实例。2026-10-02 本机测试已启动两项；当前测试期间直接运行查看器即可。

以后重启 WSL 时，在两个 WSL 终端分别启动。先进入仓库的 `ros2_ws`，各终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

终端一：

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:="$PWD/src/livox_ros_driver2/config/MID360s_config.json" \
  -p cmdline_input_bd_code:=livox0000000001
```

终端二：

```bash
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$PWD/src/FAST_LIO_ROS2/config/mid360s_drone.yaml" \
  -p use_sim_time:=false
```

Windows VSCode 选择 `livox` 解释器，直接运行 `python_sdk/lio_trajectory_viewer.py`。本地 `.vscode/settings.json` 默认解释器也已配置，此文件不提交。若 VSCode 已记住其他解释器，使用“Python: Select Interpreter”选择上述路径。

窗口显示相对首次新鲜 TRACKING 样本的 XYZ 米坐标：启动时机头向前为 X 正，左为 Y 正，重力反方向为 Z 正。按钮只重置显示原点。关闭窗口会停止它创建的 WSL 订阅；驱动和 FAST-LIO 继续运行。查看器不会自动启动驱动或任务。

未加载复核过的安装外参时，状态显示 `LIDAR ORIGIN`，坐标对应雷达原点。可用 `--mount 路径` 指定现有安装文件；不可把显示原点用于 Navigation 标定。源数据仍按原来的 50 ms 新鲜度门控，Windows 的 250 ms 管道超时只用于窗口断流显示。

## 已验证与边界（2026-10-02）

- 指定 Conda 环境 `pip check` 通过，Tk 窗口及三维预览通过。
- 查看器 6 项纯逻辑测试、`py_compile`、`git diff --check` 通过。
- 6 秒实机被动采样：高频定位 1205 条（约 201 Hz），健康消息 1199 条（约 200 Hz），1199 对配对；定位 P99 1.88 ms、健康 P99 1.93 ms，零条超过 50 ms。边界有 6 条待配对。
- Windows 查看器实际收到 TRACKING 三维坐标；显示归零后恢复更新；订阅线程退出、WSL 子进程正常退出（返回码 0）。实时窗口已打开并响应。

此结果只验证本机接收、静态定位和显示。动态尺量精度、长时间视觉并行负载、断流控制及飞行未验证。未修改 `python_sdk/FlightController/**`。

## 撤回网络设置

管理员 PowerShell 可移除本次专用规则：

```powershell
Remove-NetFirewallRule -Name MID360S-Local-UDP
Remove-NetFirewallHyperVRule -Name MID360S-WSL-UDP
```

修改前没有 `.wslconfig`。若以后要恢复旧网络模式，移除本次两个配置项并在可关闭 WSL 会话时执行 `wsl --shutdown`；保留以后新增的其他配置。
