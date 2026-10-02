# 无人机端轨迹窗口与任务 FAST-LIO 重启接口

## 无人机轨迹程序

在小电脑 NoMachine 图形桌面或 VSCode 中直接运行：

```bash
python3 python_sdk/drone_lio_trajectory_viewer.py
```

程序自动加载本机 ROS Humble 和仓库 overlay，复用 `lio_trajectory_viewer.py` 的坐标转换、轨迹断点、NumPy 绘制和显示归零。XYZ 为米，启动机头向前、左、重力反方向分别为正；无复核安装外参时显示雷达原点。

无人机端定位由 `mid360s-driver.service`、`mid360s-fastlio.service` 管理。查看器只订阅现有定位；关闭查看器不会停止任务所用服务。Windows 原入口仍每次创建自己的 WSL 测试地图。不要在无人机上运行 Windows 会话管理模式或 `--new-map` 以抢占服务。

定位服务需已启动；雷达接到无人机端后，可人工执行：

```bash
sudo systemctl start mid360s-driver.service mid360s-fastlio.service
```

## 任务调用

`fastlio_control.py` 导入时不连接飞控、不创建 ROS 线程。传入任务已有的 `fc`、`navi`，不要为重启另开串口连接。

```python
from fastlio_control import restart_and_calibrate_fastlio

# 使用任务已有的 fc 和 navi；navi.start() 已安装普通 ROS 监听器。
# 飞机须锁桨，遥测新鲜，导航/定高/轨迹/速度覆盖均关闭。
result = restart_and_calibrate_fastlio(fc, navi, timeout=45)
# 成功后 result['pose'] 是新地图中经过原有门控和基点标定的快照。
# 后续是否启用导航，由任务原来的流程决定。
```

不需要立即标定时，可调用：

```python
from fastlio_control import restart_fastlio_for_task
result = restart_fastlio_for_task(fc, navi)
assert result['needs_calibration']
# 此时旧基点已作废；任务必须随后重新执行正常的地面标定。
```

两项接口只重启 `mid360s-fastlio.service`，保留驱动；驱动未运行时返回错误。调用 `sudo -n systemctl restart ...`，不会交互询问密码或保存密码。2026-10-02 已只读核验目标机 `fc` 的非交互 sudo 权限可用。

## 拒绝条件与恢复

- 飞控连接不存在、遥测超过 0.5 秒或飞机已解锁时拒绝。
- 导航、定高、轨迹或速度覆盖执行中拒绝；任务停止事件已置位时拒绝。
- 同一进程内的重启/标定调用串行保护；任务须自行串行安排解锁、飞行动作及其他进程的服务管理。
- 操作前作废旧坐标和基点；确认服务的新 InvocationID、有效 PID、active 状态后才报告重启成功。
- 联合标定接口要求已安装的 ROS 监听器及复核机体安装参数，保留 2 秒 IMU 静止、至少 10 次有效修正、50 ms 新鲜度等现有门控。
- 只接受源时间晚于本次服务重启完成时间的标定快照。超时、停止事件、权限失败或遥测失效抛出异常，不恢复旧基点、不自动启用导航。
- 任务应让异常进入原有停止/退出路径；不要捕获后继续使用旧航点坐标。飞行中重建地图不是此接口支持的恢复方式。

共享查看器检测 epoch 改变或修正序号回退时清空显示原点和轨迹，适配系统服务重建地图。

## 修改与验证

修改/新增文件：`python_sdk/drone_lio_trajectory_viewer.py`、`python_sdk/fastlio_control.py`、`python_sdk/lio_trajectory_viewer.py`、`python_sdk/FlightController/Components/LioPoseProvider.py`、`python_sdk/testcode/test_fastlio_control.py`、`python_sdk/testcode/test_lio_pose_provider.py`、`python_sdk/testcode/test_lio_trajectory_viewer.py`、本说明。

底层变更仅为 `LioPoseProvider.invalidate_for_restart()`：现有公共接口无法主动作废仍健康的旧基点，任务层不能依赖修改私有字段。新方法在原有锁下复用 `_latch_lost()`，只撤销定位许可；不改变飞控协议、坐标单位、串口、控制输出、健康阈值或原有 ground reset/calibration 接口。

本地：65 项定位/Navigation/服务门控/查看器/延迟/重启测试通过，Python 语法和 `git diff --check` 通过。测试用 stub 飞控和 systemctl，覆盖已解锁、断线、遥测过期、导航/轨迹执行、停止事件、重启失败、旧基点作废、新地图时间门控和标定超时。

无人机端实机服务重启、静止标定、新窗口显示、任务异常恢复、飞行和长时间负载需在设备具备安全条件时验证。本次代码同步不启动任务、不启动定位服务。回退时恢复本次提交的上述 Python 文件版本并通过 main 快进同步；本次没有修改系统服务或 sudo 配置。
