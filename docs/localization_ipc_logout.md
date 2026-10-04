# 2026-10-04：SSH 退出导致定位 DDS 共享内存被清理

## 已确认的故障链

机载 Ubuntu 使用 systemd 249.11，`fc` 是普通 UID 1000，`Linger=no`。
驱动、FAST-LIO 和 FC bridge 由系统服务启动，均以 `fc` 运行。
没有桌面自启动、cron 或 tmux 定位副本；这些系统服务不属于 SSH 登录会话。
logind 合并配置没有覆盖默认 `RemoveIPC=yes`。

雷达原始 UDP 数据仍到达网卡：点云 56301、IMU 56401。两个定位服务仍
active，图中可发现五个必需话题，但新建被动订阅器在 6 秒内五个话题均
收到 0 条消息。ROS domain、RMW 和本地网络配置未发现不一致。
两个节点进程的 `/proc/PID/maps` 中多处 FastDDS 共享内存标记 `(deleted)`，
对应文件已不在 `/dev/shm`。因此不能把服务 active 或话题可发现当作可飞依据。

## 修改前的独立复现

没有重启雷达、FAST-LIO 或 FC bridge，没有运行任务、解锁或控制命令。
通过两个有运行时上限的临时 systemd 服务，在登录会话之外模拟相同条件：
一个普通 `fc` 用户 Python 进程创建并持续 mmap 自己的诊断共享内存，
另一个 root strace 仅观察 logind 的 unlink/unlinkat/rmdir 调用。
这些是临时诊断单元，自动退出，没有持久自启动项。

本地时间 2026-10-04 +08:00 的日志证据：

- 12:00:26：唯一 SSH 会话 19 退出并移除；样本存在，nlink=1。
- 12:00:37.010322：logind PID 627 执行
  `unlinkat(23, "robocup-ipc-check-20261004", 0) = 0`。
- 同时 logind 删除 `/run/systemd/users/1000`。
- 12:00:37.750080：样本进程 PID 3519 仍运行，记录 exists=false、nlink=0。
- 样本继续运行至 12:00:54 后正常退出；strace 在限时后退出。

这直接证明 logind 在最后登录退出后清理了该用户系统服务仍使用的共享
内存。雷达映射在更早的会话退出后已经删除，未对那次原始删除做追踪；
当前配置、时间顺序和相同复现机制共同定位本次 DDS 不可用的原因。

对应实现见 systemd v249 `src/login/logind-user.c` 的 `user_finalize()`：
普通用户且 RemoveIPC 打开时执行 `clean_ipc_by_uid()`；默认最后会话退出
等待约 10 秒。配置重读由 `src/login/logind.c` 的 SIGHUP handler 完成。

## 修复与边界

部署 `RemoveIPC=no` drop-in，保留普通用户的服务 IPC；安装器备份旧配置，
核对合并设置并请求 logind 重读，不重启硬件服务。该设置作用于所有普通
用户，适用于专用机载计算机，需由程序退出清理 IPC 或在重启时清理。

`server_ros.py` 启动时使用独立子进程检查五个话题的真实消息及现有 LIO
配对健康条件。只对已证实的 deleted DDS memory 尝试一次地面恢复，复用
任务原有的服务停止、所有权审核、官方清理、服务启动和新实例确认流程。
恢复前拒绝其他 DDS 使用者和不可读的进程，保留全部任务校准与飞行检查。

此结论针对本次“雷达 UDP 存在但新 ROS 订阅无数据”。它不能解释或排除
此前飞行抖动、降落偏移、动力不足或糊味的其他原因；此次没有飞行验收。
