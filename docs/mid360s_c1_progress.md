# MID360S C1 采集链进度（2026-09-27）

已在锁定的 FAST-LIO 源码上准备 S 型 CustomMsg 路径，并更新项目跟踪的 `ros2_ws/patches/fast_lio_highrate.patch`。`ros2_ws/setup_localization_sources.sh` 继续从固定上游 SHA 应用同一补丁。

- `preprocess.livox_model: mid360s` 显式选择 S 路径；旧型号仍使用原 `avia_handler`。S 路径拒绝 feature extraction、无效置信度标签、非有限/盲区点；不按 `line` 分桶。保留点的 `curvature` 为原始 offset 的毫秒值。
- 根据本地 driver2 `Lddc::InitCustomMsg()` 与 `FillPointsToCustomMsg()` 核验：`header.stamp == timebase == pkg.base_time`，`offset_time` 是相对该起点的纳秒数。回调在过滤前检查点数和 150 ms 范围，保存**原始最大 offset** 为帧末；坏帧不进入 LIO 队列。
- 关闭全局 scan 发布时，body scan 的发布开关现在独立生效。body scan 的 frame 仍需在 C4 与机体安装外参一起核验。
- 将 bench 和生产模板标为 S 路径，并把生产模板的体素、匹配距离与局部 cube 改为计划首轮值；生产模板仍无实测 `T_I_B`，不能直接飞行。

离线验证：补丁在固定 FAST-LIO SHA `2fffc570a25d0df172720bac034fbdb6a13d2162` 的本地全新克隆上可正向应用；后续 C2 补丁也通过当前源码的反向应用检查。`git diff --check` 通过。计划附带的 45 项 Python 参考测试、C++ S 标签/点时间测试，以及项目已有的 8 项 LIO/Navigation stub 测试均通过。FAST-LIO 与驱动已在 WSL ROS 2 Humble 构建，实测 bag 已回放，结果见 `mid360s_c2_progress.md`。

## 实物采集（2026-09-27）

用户确认雷达已开启后，在机载机只启动现有驱动并短时订阅传感器话题；没有启动 FAST-LIO、飞控服务或任务。飞行器的桨叶状态未远程核实。

- 机载机为 Ubuntu 22.04.5、x86_64、ROS 2 Humble；`enp3s0` 为 `192.168.1.50/24`，启动前部署仓库干净。驱动识别 S 型雷达、进入 Normal 模式并启用 IMU 后，点云约 10 Hz、IMU 约 200 Hz。此前四秒无 IMU 的记录不能代表启动稳定后的数据状态。
- 10 秒单独 IMU 订阅收到 2000 条：源时间严格递增，最小/最大间隔约 3.93/6.10 ms；静置加速度模均值 0.9933（驱动原始值接近 g），最大角速度模 0.0091。应按估计器现有 `G_m_s2/mean_acc.norm()` 尺度处理。
- 点云样本 82 帧的 `header.stamp - timebase` 全为 0，点数全部匹配，原始最大点 offset 约 100.51 ms，未出现超过 150 ms 的点。两路同时用单线程 Python 遍历全部点会拖慢 IMU 回调，因此连续性结论来自独立 IMU 订阅。
- 使用 `ros2 bag record` 得到 12.63 秒数据：`/livox/lidar` 126 帧、`/livox/imu` 2527 条。本地副本位于 `C:\Users\TZDEZACR\Desktop\robocup\tmp\mid360s-c1-d7296885`，DB3 SHA256 为 `5c97d26ac6261d5930b33aaac5f5639525a93b08b105f47ffe3c459db709c607`，与机载副本一致。

**C1 采集与原始点时间合同已通过；ROS 离线集成已编译并回放，但动态精度与实时门控尚未验收。** 驱动每次按 SIGINT 关闭后 SDK 报告已释放，但节点仍以 `-7` 退出；采样后确认没有遗留驱动进程。该退出异常需在长期运行前定位。
