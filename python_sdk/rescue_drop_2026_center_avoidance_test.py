"""单独试飞航迹 1 和中心航迹 3 的避障效果。

不启动相机、视觉线程或继电器；目标点不会触发投掷状态。
默认只打印航迹，不连接硬件。真实飞行需 --confirm-flight 和终端 start_mission。
飞控使用已运行的 FC_Server，定位与点云避障沿用主任务。
"""

import argparse
import sys
import threading
import time

import rescue_drop_2026 as rescue


def run_center_avoidance_test(mission):
    """航迹 1 到终点后执行航迹 3，并在航迹 3 终点降落。"""
    if mission.vision is not None or mission.relay is not None:
        raise RuntimeError("center test must not use vision or payload relays")
    mission._check()
    navi = mission.navi
    navi.pointing_takeoff(rescue.TAKEOFF_POINT, target_height=rescue.CRUISE_HEIGHT)
    if not navi.wait_for_height(timeout=10):
        raise RuntimeError("cruise takeoff height was not confirmed")
    navi.set_yaw(0)
    if not navi.wait_for_yaw():
        raise RuntimeError("yaw stabilization was not confirmed")

    # 无检测参数：航迹 1 上的自由目标不会中断导航。
    for waypoint in mission.route_1:
        mission._navigate_leg(waypoint)
    if not mission._at_waypoint(mission._position(), mission.route_3[0]):
        raise RuntimeError("route 1 endpoint was not confirmed")

    mission.state = rescue.MissionState.CENTER
    # 复用主任务的中心点云规划与动态重规划，但不查询黄色目标。
    mission._execute_center_route(detect_mandatory=False)
    mission._check()
    endpoint = mission.route_3[-1]
    if not mission._at_waypoint(mission._position(), endpoint):
        raise RuntimeError("route 3 endpoint was not confirmed")
    if not navi.pointing_landing(
            endpoint, height_timeout=rescue.LANDING_HEIGHT_TIMEOUT):
        raise RuntimeError("pointing landing at route 3 endpoint was not confirmed")
    mission.landed = True
    rescue.logger.info("[CENTER-TEST] Landed and locked at route 3 endpoint {}", endpoint)


def parse_args():
    parser = argparse.ArgumentParser(description="2026 中心航迹 3 避障独立试飞")
    parser.add_argument("--confirm-flight", action="store_true",
                        help="启用真实飞行；默认仅打印航迹")
    parser.add_argument("--fc-host", default=rescue.FC_SERVER_HOST,
                        help="FC_Server 地址；机上服务默认在本机")
    parser.add_argument("--fc-server-port", type=int, default=rescue.FC_SERVER_PORT,
                        help="FC_Server TCP 端口，默认 5654")
    args = parser.parse_args()
    if not 1 <= args.fc_server_port <= 65535:
        parser.error("--fc-server-port must be between 1 and 65535")
    return args


def wait_for_start_command():
    rescue.logger.warning("[CENTER-TEST] No camera or relays; enter start_mission to take off")
    while True:
        if input("[CENTER-TEST] start_mission> ") == "start_mission":
            return
        rescue.logger.warning("[CENTER-TEST] Ignored command; enter exactly start_mission")


def wait_for_obstacle_map(mission, planner):
    """起飞前确认至少两次新鲜点云更新，避免无地图时先飞航迹 1。"""
    deadline = time.monotonic() + rescue.LIO_POSE_READY_TIMEOUT
    first_revision = None
    while time.monotonic() < deadline:
        mission._check()
        status = planner.get_debug_state()
        if status["ready"]:
            if first_revision is not None and status["revision"] > first_revision:
                rescue.logger.info("[CENTER-TEST] Obstacle map ready: {}", status)
                return
            first_revision = status["revision"]
        else:
            first_revision = None
        mission.stop_event.wait(0.1)
    raise RuntimeError("fresh, updating 2-D obstacle point cloud unavailable")


def main():
    args = parse_args()
    route_1, route_3, _ = rescue.build_routes()
    rescue.logger.info("[CENTER-TEST] Route 1: {}", route_1)
    rescue.logger.info("[CENTER-TEST] Route 3: {}", route_3)
    rescue.logger.info("[CENTER-TEST] Landing point: {}", route_3[-1])
    if not args.confirm_flight:
        rescue.logger.warning("[CENTER-TEST] Preview only; no hardware connected")
        return 0

    stop_event = threading.Event()
    fc = None
    navi = None
    mission = None
    takeoff_attempted = False
    try:
        if not rescue.ObstacleInterface.IMPLEMENTED:
            raise RuntimeError("center obstacle avoidance is not implemented")
        obstacle_planner = rescue.ObstaclePlanner2D()
        obstacle = rescue.ObstacleInterface(obstacle_planner)

        fc = rescue.FC_Client()
        fc.connect(host=args.fc_host, port=args.fc_server_port, authkey=b"fc",
                   print_state=False, block=True, timeout=10)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("FC_Server connection timeout")
        telemetry_deadline = time.monotonic() + 10.0
        while not fc.state.is_fresh(0.5):
            if not fc.connected or time.monotonic() >= telemetry_deadline:
                raise RuntimeError("fresh flight-controller telemetry unavailable via FC_Server")
            stop_event.wait(0.05)
        if fc.state.unlock.value:
            raise RuntimeError("flight controller already unlocked; refuse takeover")

        navi = rescue.Navigation(fc=fc, stop_event=stop_event,
                                 obstacle_planner=obstacle_planner,
                                 height_source="lio")
        mission = rescue.Mission(
            fc, navi, None, None, obstacle,
            {color: 0 for color in rescue.FREE_COLORS}, stop_event)
        mission.prepare_navigation()
        wait_for_obstacle_map(mission, obstacle_planner)
        wait_for_start_command()
        mission.deadline = time.monotonic() + rescue.MISSION_TIMEOUT
        mission._check()
        wait_for_obstacle_map(mission, obstacle_planner)
        takeoff_attempted = True
        run_center_avoidance_test(mission)
        return 0
    except KeyboardInterrupt:
        rescue.logger.warning("[CENTER-TEST] Interrupted by user")
        return 130
    except Exception:
        rescue.logger.exception("[CENTER-TEST] Flight failed")
        return 1
    finally:
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                rescue.logger.exception("[CENTER-TEST] Failed to stop mission")
        elif navi is not None:
            try:
                navi.stop()
            except Exception:
                rescue.logger.exception("[CENTER-TEST] Failed to stop navigation")

        if (takeoff_attempted and fc is not None and fc.connected
                and fc.state.unlock.value):
            try:
                rescue.emergency_land(fc)
            except Exception:
                rescue.logger.exception("[CENTER-TEST] Emergency landing request failed")

        if navi is not None:
            try:
                rescue.stop_navigation_ros(navi)
            except Exception:
                rescue.logger.exception("[CENTER-TEST] Failed to stop LIO ROS executor")
        if fc is not None:
            try:
                fc.close()
            except Exception:
                rescue.logger.exception("[CENTER-TEST] Failed to close flight controller")


if __name__ == "__main__":
    sys.exit(main())
