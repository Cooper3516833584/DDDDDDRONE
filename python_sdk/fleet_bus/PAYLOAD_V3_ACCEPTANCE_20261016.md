# TASK v3 / 黄色一件：离线验收（2026-10-16）

## 实际变更与基线

重新fetch后远端main为 `3a18362e64fddb4db9394fe5c748e18fc95fac5d`；本地另有 `4a6bd7f2b1462304bdedf217cbd115ef2fee0593` 飞控接收修复。
本地合并提交 `94cb306` 保留两侧历史并纳入远端地面链路测试入口；本次配额功能未修改 `python_sdk/FlightController/**`。

- TASK/START语义版本3，RGB仅接受非bool整数全零，隐含黄色一件；START独立校验task_id，拒绝旧v2，结构仍为 `<BI3B` / `<BI`。
- FREE_DROP_COUNT=0、MANDATORY_DROP_COUNT=1、TOTAL_DROP_COUNT=1；手动实飞要求RGB三个0，车控入口允许零自由配额。
- PAYLOAD_COUNT=1、DROP_SERVO_SEQUENCE=(2,)，暂定黄色144°槽位/STM32通道3。这是编码选择，尚未确认机械装填；如实际在原首槽，应改为(0,)/通道2并重验，只能选一槽。第二次释放被拒绝。
- 物理四路输出、all open/close、四路关闭检查、位置映射、原航迹/避障/FC锁桨/人工装填/FAST-LIO/相机/急停及落地清理保留。RGB全零时视觉跟踪只启用黄色。
- CLOSE/REMAP/session/run_id/幂等ACK机制保持，FleetBus外层仍V1。
- `.gitignore`仅增加本次测试反复生成的testcode/fc_log目录，不删除日志。

## 已执行验证

Windows Python3.13。命令均从无人机仓库根目录运行；模块路径使用PowerShell `$env:PYTHONPATH='python_sdk'`，对应Linux `PYTHONPATH=python_sdk`。

| 命令 | 结果 |
|---|---|
| `python -m compileall -q python_sdk` | 退出0，无语法错误；提示无法遍历既有 `.pytest_cache` |
| `python -m unittest discover -s python_sdk/testcode -p 'test_*.py'` | 运行301项；0断言失败，16个导入/旧文件错误，未宣称全量通过 |
| `python -m unittest discover -s python_sdk/testcode -p 'test_rescue*.py'` | 43项通过，包含地面链路与任务/入口/释放 |
| `python -m unittest discover -s python_sdk/testcode -p 'test_payload_servos.py'` | 6项通过 |
| `python -m unittest discover -s python_sdk/testcode -p 'test_fleet_protocol.py'` | 6项通过 |
| `python -m unittest discover -s python_sdk/testcode -p 'test_air_fleet_node.py'` | 26项通过 |
| `python -m unittest discover -s python_sdk/testcode -p 'test_obstacle_planner_2d.py'` | 77项，76通过，1错误：既有NumPy/SciPy `cannot load module more than once per process` |
| `git diff --check` | 通过 |

核心相关43+6+6+26=81项全部通过，无新增跳过。避障测试保留原失败覆盖，未修改底层样条或依赖。
小车仓库 `python tools/verify_payload_handoff.py --drone-repo ../drone` 的9组模拟全部通过，覆盖三种任务牌×三个任务ID边界。
实际双端编解码与收发器、CLOSE/REMAP、TASK COMPLETED后START ACCEPTED及机端仅黄色一次释放均得到验证；全部硬件接口为假对象。

全量16个错误详情（没有删除或放宽这些测试）：

- `test_camera_fov`、`test_legacy_pointing_takeoff`、`test_navigation_direct_cruise`：缺少simple_pid。
- `test_disaster_survey_electromagnet`、`test_mission1_cue_states`、`test_mission1_fleet_pose_reporting`、`test_mission2_26_logic`、`test_mission2_cue_states`、`test_landing_marker_altitude_monitor`、`test_moving_target_descent_logic`、`test_screen_manager`、`test_task2_route_plan`：引用当前检出中缺失的旧模块/入口。
- `test_relay_lcus`、`test_relay_lcus_sequence`、`test_screen_comm`、`test_smooth_trajectory_flight`：FlightController旧组件缺失或同进程测试桩覆盖包导致导入不可用。

## 交付与未验证范围

本次功能仅本地提交，未推送、未SSH部署、未重启设备或服务、未运行真实飞行/舵机/相机/雷达。
未修改 `python_sdk/FlightController/**`（本次配额补丁相对合并基线）；合并保留的4a6bd7f底层修复属于原有提交。
黄色实际装填、STM32物理动作、车端PWM6 overlay/接线/脉宽隔离、无线链路与两端实机验收全部待操作者现场验证。两端须同步协议v3后再正式联机。

## 修改文件

- `.gitignore`
- `python_sdk/fleet_bus/RESCUE_TASK.md`
- `python_sdk/fleet_bus/PAYLOAD_V3_ACCEPTANCE_20261016.md`
- `python_sdk/fleet_bus/drop_task_protocol.py`
- `python_sdk/payload_servos.py`
- `python_sdk/rescue_drop_2026.py`
- `python_sdk/testcode/test_fleet_protocol.py`
- `python_sdk/testcode/test_obstacle_planner_2d.py`
- `python_sdk/testcode/test_payload_servos.py`
- `python_sdk/testcode/test_rescue_car_entry.py`
- `python_sdk/testcode/test_rescue_payload.py`
- `python_sdk/testcode/test_rescue_task_link.py`
