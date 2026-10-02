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

Windows 直接运行查看器会自动启动本机 WSL 中的新 FAST-LIO 实例，重新建立地图和显示原点。若驱动尚未运行，也会自动启动驱动。关闭窗口或在其终端 Ctrl+C 后，停止本次启动的 FAST-LIO 和驱动。已有的外部驱动由原启动者管理。

启动时保持雷达静止，等待初始化完成和 `TRACKING` 后再移动。本机冷启动实测可能需要几十秒。再次打开窗口时会生成新地图；同时打开第二个窗口会报错并要求先关闭第一个，保护已有会话。

Linux/小电脑直接运行默认仍订阅现有 ROS 服务。需要单独测试建图时可显式使用 `--new-map`，但必须先在原终端或服务中停止已有 FAST-LIO。脚本不会强行关闭外部定位服务。

### 可选：手动管理定位链

Windows 可使用 `--subscribe-only` 仅查看已有地图。此模式下，驱动与 FAST-LIO 各在一个 WSL 终端启动，进入仓库的 `ros2_ws`，各终端执行：

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

### Windows 查看器入口

Windows VSCode 选择 `livox` 解释器，直接运行 `python_sdk/lio_trajectory_viewer.py`。本地 `.vscode/settings.json` 默认解释器也已配置，此文件不提交。若 VSCode 已记住其他解释器，使用“Python: Select Interpreter”选择上述路径。

Windows PowerShell 和 PowerShell 7 的用户 `profile.ps1` 已加入 Conda 初始化。配置后关闭旧终端、打开新终端，再运行：

```powershell
conda activate livox
python .\python_sdk\lio_trajectory_viewer.py
```

旧终端可先执行 `(& 'C:\Users\TZDEZACR\miniconda3\Scripts\conda.exe' 'shell.powershell' 'hook') | Out-String | Invoke-Expression`，然后激活。查看器固定使用 Tk 独立窗口，并等待窗口关闭；不依赖 VSCode 的 inline/interactive 后端设置。

窗口显示相对首次新鲜 TRACKING 样本的 XYZ 米坐标：启动时机头向前为 X 正，左为 Y 正，重力反方向为 Z 正。按钮只重置显示原点。Windows 默认模式关闭窗口会停止订阅和本次启动的定位节点。脚本不启动飞行任务。

未加载复核过的安装外参时，状态显示 `LIDAR ORIGIN`，坐标对应雷达原点。可用 `--mount 路径` 指定现有安装文件；不可把显示原点用于 Navigation 标定。源数据仍按原来的 50 ms 新鲜度门控，Windows 的 250 ms 管道超时只用于窗口断流显示。

## 已验证与边界（2026-10-02）

- 指定 Conda 环境 `pip check` 通过，Tk 窗口及三维预览通过。
- 查看器 9 项纯逻辑测试（包括启动失败清理、进程退出升级）、`py_compile`、`git diff --check` 通过。
- 6 秒实机被动采样：高频定位 1205 条（约 201 Hz），健康消息 1199 条（约 200 Hz），1199 对配对；定位 P99 1.88 ms、健康 P99 1.93 ms，零条超过 50 ms。边界有 6 条待配对。
- Windows 查看器实际收到 TRACKING 三维坐标；显示归零后恢复更新；订阅线程退出、WSL 子进程正常退出（返回码 0）。实时窗口已打开并响应。
- 会话管理版本通过 `ros2 run` 自动启动驱动和新地图；50 秒受控测试内连续多次收到 TRACKING 坐标。退出后核验无驱动、FAST-LIO、订阅残留。重复窗口测试被锁拒绝，原会话不被停止。
- 原生 ROS 节点退出时仍可能输出 publisher/datawriter 销毁错误；本次清理已确认进程退出，但未修复原生节点的退出诊断。

此结果只验证本机接收、静态定位和显示。动态尺量精度、长时间视觉并行负载、断流控制及飞行未验证。未修改 `python_sdk/FlightController/**`。

### LOST 且 XYZ 显示空值

若窗口一直显示 `LOST - laser_correction_stale`，表示 FAST-LIO 已收到 IMU，但有效点云修正过期；查看器没有有效 TRACKING 原点时保持空值。2026-10-02 实测曾出现持续 `No Effective Points!`，停止旧 FAST-LIO 并重新建图后恢复 TRACKING；复测健康消息为 `geometry_pass`，有效匹配点 719。具体诱因尚未确认。

保持雷达静止、周围有可扫描的场景，关闭窗口并重新运行即可重新建图。手动订阅模式需在 FAST-LIO 所属终端 Ctrl+C 后重新执行终端二命令。“Reset display origin”仅重置显示，不能修复 FAST-LIO 的地图匹配。健康门控仍然保留。

### 之前的后台实例从哪里启动、如何关闭

2026-10-02 之前持续运行的本机定位实例由 Codex 工具终端执行 `wsl.exe ... ros2 run ...` 手动启动，与查看器窗口分开；本机未启用 `mid360s-fastlio.service` 或 `mid360s-driver.service`。改为会话管理前已核验并停止这些手动实例。

新默认模式关闭查看器窗口即可退出 FAST-LIO。手动模式在启动 FAST-LIO 的终端按 Ctrl+C。若终端丢失，先用 `wsl --exec ps -eo pid,args` 核验具体进程，再 `wsl --exec kill -INT 实际PID`。不要沿用旧 PID，也不要用模糊进程匹配结束其他任务。

## 撤回网络设置

管理员 PowerShell 可移除本次专用规则：

```powershell
Remove-NetFirewallRule -Name MID360S-Local-UDP
Remove-NetFirewallHyperVRule -Name MID360S-WSL-UDP
```

修改前没有 `.wslconfig`。若以后要恢复旧网络模式，移除本次两个配置项并在可关闭 WSL 会话时执行 `wsl --shutdown`；保留以后新增的其他配置。
