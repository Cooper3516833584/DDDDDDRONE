# MID360S C1 采集链进度（2026-09-27）

已在锁定的 FAST-LIO 源码上准备 S 型 CustomMsg 路径，并更新项目跟踪的 `ros2_ws/patches/fast_lio_highrate.patch`。`ros2_ws/setup_localization_sources.sh` 继续从固定上游 SHA 应用同一补丁。

- `preprocess.livox_model: mid360s` 显式选择 S 路径；旧型号仍使用原 `avia_handler`。S 路径拒绝 feature extraction、无效置信度标签、非有限/盲区点；不按 `line` 分桶。保留点的 `curvature` 为原始 offset 的毫秒值。
- 根据本地 driver2 `Lddc::InitCustomMsg()` 与 `FillPointsToCustomMsg()` 核验：`header.stamp == timebase == pkg.base_time`，`offset_time` 是相对该起点的纳秒数。回调在过滤前检查点数和 150 ms 范围，保存**原始最大 offset** 为帧末；坏帧不进入 LIO 队列。
- 关闭全局 scan 发布时，body scan 的发布开关现在独立生效。body scan 的 frame 仍需在 C4 与机体安装外参一起核验。
- 将 bench 和生产模板标为 S 路径，并把生产模板的体素、匹配距离与局部 cube 改为计划首轮值；生产模板仍无实测 `T_I_B`，不能直接飞行。

离线验证：补丁在固定 FAST-LIO SHA `2fffc570a25d0df172720bac034fbdb6a13d2162` 的本地全新克隆上可正向应用，应用后 5 个修改文件的 SHA256 与当前开发源码逐一一致。`git diff --check` 通过。计划附带的 45 项 Python 参考测试通过；项目已有的 8 项 LIO/Navigation stub 测试通过。当前 Windows 环境无 ROS 2 编译工具链或 C++ 编译器，未声称 C++ 编译通过。

**C1 未完成**：`ros2_ws/README.md` 中最近一次实物检查记录 `/livox/lidar` 约 9.98 Hz，但 `/livox/imu` 四秒内无消息。需要在机载机独占驱动、桨叶拆除且雷达固定的条件下，检查 `ros2 topic info -v`、`ros2 topic hz`、驱动和 SDK 的 IMU 原始数据、端口接收以及 10 秒时间连续性；确认真实约 200 Hz IMU、点时间覆盖且驱动可正常退出后，才可标记 C1 通过。此轮未执行这些现场操作。
