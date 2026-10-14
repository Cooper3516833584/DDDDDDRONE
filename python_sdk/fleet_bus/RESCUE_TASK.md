# 小车控制的救援投放配额

任务板是红、蓝、绿地块三行数量，合计 4 件自由投放物资，全部由无人机执行。
`rescue_drop_2026.py` 新增 `--car-control`，通过现有 FC_Server/飞控 UART2 FleetBus 接收小车实际配额及匹配的启动请求；不使用机载 CH340（该口是继电器）。

`0x26 DRONE_SET_DROP_TASK` body 为 `<BI3B`：版本 1、非零 task_id、红/蓝/绿数量。校验并保存成功才回复 `COMPLETED`。
`0x27 DRONE_START_DROP_TASK` body 为 `<BI`：版本 1、相同 task_id。只接受与已保存数据相同的小车 session/任务编号，回复 `ACCEPTED`。
源必须为小车 `0x20`，目标必须为无人机 `0x10`。地面站直接命令不会触发此接收器。合法配额每色 0～4、合计 4，黄色必投沿用现有固定 1 件。

`rescue_task.py` 的无线回调只进入 FleetBus 队列，工作线程保存配额/启动标记，不控制导航或执行器。主入口等待匹配开始后，将实际数量传入 `DropLedger`，随后执行原任务。重复帧复用 ACK 缓存，重复开始不会重置账本。
收到 `TARGETED_STOP` 后设置任务 stop_event，仅回复已接受；实际着陆依赖主程序现有清理路径，ACK 不代表已经落地。

```bash
# 接收/记录任务数据，原定位监视模式，不允许无线启动飞行
python3 python_sdk/rescue_drop_2026.py --car-control
```

实际比赛需在人工确认环境、定位、视觉和载荷后，使用 `--car-control --confirm-flight` 与确认的继电器串口。此模式不得同时传命令行颜色数量，正式配额仅来自小车。
手动模式仍使用 `--red-count/--blue-count/--green-count` 和终端 `start_mission`。
`rescue_drop_2026_clear_area.py` 共用上述参数和通信入口，仅适用于人工确认全航段净空的场地。
改变自由投放总数的测试入口（例如三件自由物资、两件黄色物资的 two-yellow 测试）拒绝 `--car-control`，避免误用正式任务板配额。

通信接收不会自动安装服务或重启任务。双方结束后重启待命、地面站到小车常驻接收和整个任务自启动仍待接入。本次测试不进行真实起飞、落地或投放。

无硬件测试：

```bash
PYTHONPATH=python_sdk python3 -m unittest discover -s python_sdk/testcode -p 'test_rescue_*.py'
PYTHONPATH=python_sdk python3 -m unittest discover -s python_sdk/testcode -p 'test_air_fleet_node.py'
```

## 本次修改与验证（2026-10-11）

修改文件：`python_sdk/rescue_drop_2026.py`、`python_sdk/fleet_bus/air_node.py`、`python_sdk/fleet_bus/models.py`、`python_sdk/fleet_bus/drop_task_protocol.py`、`python_sdk/fleet_bus/rescue_task.py`、`python_sdk/fleet_bus/README.md`、本文、`python_sdk/testcode/test_rescue_task_link.py`、`python_sdk/testcode/test_rescue_car_entry.py`、`python_sdk/testcode/test_obstacle_planner_2d.py`。

未修改 `python_sdk/FlightController/**`。
修改 Python 文件的 `py_compile` 与 `git diff --check` 通过；新接收/入口测试 8 项、既有 AirFleetNode 测试 26 项通过；与小车两端实际代码的 45 组无硬件联测通过。
合入远端已有的任务更新后，避障和主入口扩展回归执行 67 项，其中 66 项通过；既有 `test_real_spline_can_leave_a_clear_polyline` 被本机 NumPy 2.5.1 / SciPy 1.15.3 的加载错误 `cannot load module more than once per process` 阻断。单独执行该测试也失败；该测试方法及其被调用的底层文件未被本次通信提交修改。未更改本机或设备的依赖版本。
按用户要求仅推送 GitHub，本次不拉取设备、不启动飞行程序。真实无线链路、距离/干扰、起飞、落地、投放和双方重启待命未做验证。
