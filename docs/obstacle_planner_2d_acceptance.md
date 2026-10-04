# 二维雷达避障实现与验收记录

日期：2026-10-04。基线：`main @ e5e287fbcd3f2d4cb430136fa55f2ba1466a6100`。
实现依据：用户提供的 `robocup_obstacle_planner_package/IMPLEMENTATION_PLAN.md` 和 starter。

## 已完成的步骤

1. 新增 `python_sdk/FlightController/Components/ObstaclePlanner2D.py`，复用 starter 单文件设计：24 m 地图、10 cm 栅格、水平切片、射线积分、60 cm 膨胀（按用户要求由 45 cm 调整）、禁止斜穿障碍角的八邻域 A*、路径简化和稳定子目标缓存。
2. `python_sdk/FlightController/Components/RosNode.py` 的 `LioListenNode.__init__()` 新增默认 `None` 的点云回调，通过既有节点和 `RosNodeRunner` 订阅 `/cloud_registered_body`，QoS 使用 `qos_profile_sensor_data`。
3. `python_sdk/FlightController/Solutions/Navigation.py` 的 `__init__()` 可选绑定规划器到 `LioPoseProvider.get_pose()`；`start()` 把点云回调传入原监听器。
4. `python_sdk/rescue_drop_2026.py` 的 `ObstacleInterface` 实现薄适配，`IMPLEMENTED=True`。正式目标无路时保留 `None`，过期异常向现有任务停止/降落链传播。
5. 比赛入口 `main()` 仅在 `--confirm-flight` 时创建一份规划器，供 Navigation 和任务适配器共用。监视模式不创建规划器。
6. 新增 `python_sdk/testcode/test_obstacle_planner_2d.py`，使用纯算法、AST 抽取及替身检查，不连接硬件。

用户随后要求膨胀半径从 45 cm 改为 60 cm。调整后合并执行新增测试和下列六个现有回归文件，结果为 105 项通过、13 项 subtests 通过；compileall 和 diff 检查通过。同步保留当前 main 中另一项已提交的相机设置改动 `41e270f`：index=0、640×480、30 FPS。

## 对 starter 的必要修正

- 点云过期检查提前到速度大小判断之前：零速和极低速也遵守 0.6 s 超时契约。
- 正式目标在膨胀图中被占用时，提前返回无路，避免复用旧子目标或起点临时清空绕过目标检查。
- 临时速度目标寻找 free cell 时先查询原膨胀图，再清空起点区域；否则完全被包围时会错误地把起点清空区域作为出口，返回非零速度。新增测试复现了此问题，修正后通过。
- 离线输入的 pose 若明确标为不可用，不刷新地图和点云新鲜时间，与 ROS 入口行为一致。

## 底层控制代码修改说明

已修改 `python_sdk/FlightController/**`：新增规划组件，并局部修改 `LioListenNode.__init__()`、`Navigation.__init__()` 和 `Navigation.start()`。
原因：任务层无法独自接入现有 ROS 监听器和经过校准的 LIO 位姿；使用上述可选接口能复用原执行器。
飞控协议、串口、ACK、PID、控制输出单位、线程启动和任务状态机保持原有行为。规划输入为 body 点云（m），输出为 startup-local 航点（cm）和速度（cm/s），yaw 为顺时针正。
非避障调用仍可使用原参数；FAST-LIO 配置与 patch 未修改。回退时撤回这次四个生产文件的改动，入口恢复原来的未实现避障检查。

## 已执行验证

本地 Windows / Python 3.13.12 / NumPy 2.5.1，无 ROS 运行环境。

从仓库根目录执行：

```powershell
python -B -m unittest discover -s python_sdk/testcode -p test_obstacle_planner_2d.py -v
```

结果：22 项通过，覆盖空地图、绕柱、路径不进入膨胀图、缓存稳定/失效、被占用目标、斜角穿越、yaw/平移、切片/距离过滤、积分饱和/清障、失效 pose、缺失/过期点云、速度转向/无路归零、越界、reset、ROS 解码与限频、订阅/Navigation/比赛入口接线。

相关现有离线回归：

```powershell
python -B -m pytest -q python_sdk/testcode/test_navigation_lio_integration.py python_sdk/testcode/test_navigation_cpu_affinity.py python_sdk/testcode/test_lio_pose_provider.py python_sdk/testcode/test_fast_lio_patch_contract.py python_sdk/testcode/test_fastlio_control.py python_sdk/testcode/test_lio_waypoint_flight_logic.py -p no:cacheprovider --basetemp=C:/Users/TZDEZACR/Desktop/robocup/tmp/obstacle-planner-pytest
```

结果：83 项通过，5 项 subtests 通过。仅执行明确不触发硬件的相关回归，未运行全仓库硬件测试入口。
功能包原始 smoke test 也通过，运行时使用 Components 目录作为 `PYTHONPATH`。

静态检查：

```powershell
python -m py_compile python_sdk/FlightController/Components/ObstaclePlanner2D.py python_sdk/FlightController/Components/RosNode.py python_sdk/FlightController/Solutions/Navigation.py python_sdk/rescue_drop_2026.py python_sdk/testcode/test_obstacle_planner_2d.py
python -m compileall -q python_sdk/FlightController/Components python_sdk/FlightController/Solutions python_sdk/rescue_drop_2026.py python_sdk/testcode/test_obstacle_planner_2d.py
git diff --check
```

## 未完成：真机无桨验收

本次未主动启动定位链、未连接飞控/相机/继电器、未试飞。Git 代码同步不代表硬件验收。以下项目需在拆桨且设备环境已确认后执行。

- [ ] 确认生产安装外参已实测并 `reviewed: true`，LIO 校准成功，定位 topic 持续更新。
- [ ] `/cloud_registered_body` 和 `/Odometry_highrate` 持续输出；前者需来自已应用现有 patch 的 FAST-LIO。
- [ ] 同高度障碍使 `get_debug_state()` 显示 `ready=True`、`occupied_cells>0`、`inflated_cells>occupied_cells`、`cloud_age_s<0.6`，地板不会填满地图。
- [ ] 手持平移、yaw=±90° 旋转时，同一障碍在 startup-local 图中保持固定，无镜像。
- [ ] 停止点云后，0.6 s 后 `safe_waypoint()` 和所有速度大小的 `safe_velocity()` 均抛 `RuntimeError`；任务停止水平运动并进入既有失败处置。

Topic 检查命令（路径按设备实际部署目录）：

```bash
source /opt/ros/humble/setup.bash
source ~/dddddrone/ros2_ws/install/setup.bash
ros2 topic hz /cloud_registered_body
ros2 topic echo /cloud_registered_body --once
ros2 topic hz /Odometry_highrate
```

若 body 点云缺失，依次核对现有源码 setup/patch、ROS workspace build、实际配置的 `scan_bodyframe_pub_en: true`。

## 未完成：首次带桨验收与边界

完成无桨验收后，按功能包顺序人工安排：10–15 cm/s 单根软柱 protected 航段，再两/三根障碍，最后完整 CENTER 和黄色目标接近。确认导航 worker 不会每 0.1 s 重启。

当前设计是二维平面避障，unknown 当 free，使用当前 fresh LIO 位姿转换 body 点云；实际点云时延、安装偏移、机体倾斜、60 cm 膨胀余量及控制超调仍需实测。地图和缓存须与同一 startup-local 原点一致；更换原点或定位地图后需重新初始化规划器。没有新增禁飞区规则或跨高度规划。
