"""2026 救援任务地面链路测试：收到小车 START 后停在解锁前。

在机载 Linux 的交互式 SSH 终端运行：
    python3 -u python_sdk/rescue_ground_chain_test.py --confirm-ground-test

复用正式任务的 FC_Server、STM32 装填、FAST-LIO、雷达点云和 FleetBus
CLOSE/REMAP/TASK/START 流程。相机仅采集原始帧，不加载识别模型或推理。
本入口没有解锁、起飞、航点导航、投放或降落调用。START 会被回复 ACCEPTED，
随后测试进程退出；小车后续若等待飞行结果，可能因测试模式而超时。
"""

import argparse
import signal
import sys
import threading
import time

from loguru import logger

from rescue_drop_2026 import (
    FC_Client, Navigation, ObstacleInterface, ObstaclePlanner2D,
    STM32Servos, VisionInterface, Mission, DropLedger, FREE_COLORS,
    FC_SERVER_HOST, FC_SERVER_PORT, PAYLOAD_COUNT, TOTAL_DROP_COUNT,
    connect_stm32_until_ready, stop_navigation_ros, validate_allocation,
    wait_for_operator_command,
)


def parse_args():
    parser = argparse.ArgumentParser(description="无人机锁桨状态下测试空地协同链路")
    parser.add_argument("--confirm-ground-test", action="store_true",
                        help="明确启用真实相机、雷达、无线通信和 STM32 装填动作")
    parser.add_argument("--fc-host", default=FC_SERVER_HOST)
    parser.add_argument("--fc-server-port", type=int, default=FC_SERVER_PORT)
    parser.add_argument("--start-timeout", type=float, default=900.0,
                        help="等待小车匹配 START 的秒数，默认 900")
    args = parser.parse_args()
    if not args.confirm_ground_test:
        parser.error("requires --confirm-ground-test")
    if not 1 <= args.fc_server_port <= 65535:
        parser.error("--fc-server-port must be between 1 and 65535")
    if not 0 < args.start_timeout < float("inf"):
        parser.error("--start-timeout must be finite and positive")
    return args


def require_locked(fc, stop_event):
    if stop_event.is_set():
        raise RuntimeError("ground test stopped")
    if not fc.connected or not fc.state.is_fresh(0.5):
        raise RuntimeError("FC_Server connection or telemetry lost")
    if fc.state.unlock.value:
        raise RuntimeError("flight controller unlocked; ground test stopped")


def check_camera_frames(vision, fc, stop_event, timeout=10.0):
    """沿用正式相机配置，只读原始帧，不调用 poll()/prime()/detect()。"""
    require_locked(fc, stop_event)
    vision.open()
    camera = vision._camera
    deadline = time.monotonic() + timeout
    good_frames = 0
    while good_frames < 3 and time.monotonic() < deadline:
        require_locked(fc, stop_event)
        ok, frame = camera.read()
        if ok and frame is not None and len(frame.shape) >= 2 and min(frame.shape[:2]) > 0:
            good_frames += 1
        else:
            good_frames = 0
        if good_frames < 3:
            stop_event.wait(0.1)
    if good_frames < 3:
        raise RuntimeError("target camera did not supply three consecutive raw frames")
    logger.info("[GROUND-CHAIN] Target camera raw frames ready: {}x{}",
                frame.shape[1], frame.shape[0])


def wait_for_new_pointcloud(planner, previous_revision, fc, navigation,
                            stop_event, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        require_locked(fc, stop_event)
        state = planner.get_debug_state()
        if (navigation.pose_is_fresh() and navigation.height_is_fresh()
                and state["ready"] and state["revision"] > previous_revision
                and state["last_endpoint_cells"] >= state["min_valid_endpoint_cells"]):
            logger.info("[GROUND-CHAIN] New radar cloud ready: revision={} "
                        "age={:.2f}s endpoints={} occupied={}",
                        state["revision"], state["cloud_age_s"],
                        state["last_endpoint_cells"], state["occupied_cells"])
            return
        stop_event.wait(0.2)
    raise RuntimeError("no fresh valid radar point cloud after FAST-LIO rebuild")


def wait_for_car_start(receiver, fc, stop_event, timeout):
    deadline = time.monotonic() + timeout
    while True:
        require_locked(fc, stop_event)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("matching car START was not received before timeout")
        try:
            return receiver.wait_for_start(timeout=min(0.5, remaining))
        except TimeoutError:
            pass


def main():
    args = parse_args()
    if sys.platform != "linux" or not sys.stdin.isatty():
        raise RuntimeError("ground test requires an interactive Linux terminal")
    if not 1 <= TOTAL_DROP_COUNT <= PAYLOAD_COUNT:
        raise RuntimeError("drop count exceeds available payload slots")

    stop_event = threading.Event()
    previous_sigterm = signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    fc = None
    navi = None
    payload = None
    vision = None
    mission = None
    fleet_node = None
    task_receiver = None
    try:
        fc = FC_Client()
        fc.connect(host=args.fc_host, port=args.fc_server_port, authkey=b"fc",
                   print_state=False, block=True, timeout=10)
        if not fc.wait_for_connection(timeout_s=10):
            raise RuntimeError("FC_Server connection timeout")
        telemetry_deadline = time.monotonic() + 10.0
        while not fc.state.is_fresh(0.5):
            if stop_event.is_set() or not fc.connected or time.monotonic() >= telemetry_deadline:
                raise RuntimeError("fresh flight-controller telemetry unavailable")
            stop_event.wait(0.05)
        require_locked(fc, stop_event)

        payload = STM32Servos()
        connect_stm32_until_ready(payload, fc, stop_event)
        require_locked(fc, stop_event)
        payload.open_all()
        logger.info("[GROUND-CHAIN] STM32 ALL OPEN ACK; no mechanical feedback")
        wait_for_operator_command("loaded", stop_event)
        require_locked(fc, stop_event)
        payload.close_all()
        if payload.get_states() != {channel: False for channel in range(1, 5)}:
            raise RuntimeError("STM32 did not report all four target states closed")
        logger.info("[GROUND-CHAIN] STM32 ALL CLOSED ACK; no mechanical feedback")

        vision = VisionInterface()
        check_camera_frames(vision, fc, stop_event)
        wait_for_operator_command("position_ok", stop_event)
        require_locked(fc, stop_event)

        planner = ObstaclePlanner2D()
        obstacle = ObstacleInterface(planner)
        navi = Navigation(fc=fc, stop_event=stop_event,
                          obstacle_planner=planner, height_source="lio")
        navi.start()
        previous_revision = planner.get_debug_state()["revision"]
        from fastlio_control import restart_and_calibrate_fastlio

        preflight_result = restart_and_calibrate_fastlio(fc, navi, timeout=45)
        if (preflight_result.get("needs_calibration") is not False
                or not preflight_result.get("pose")):
            raise RuntimeError("new FAST-LIO pose/calibration is not ready")
        allocation = {color: 0 for color in FREE_COLORS}
        mission = Mission(fc, navi, payload, vision, obstacle, allocation, stop_event)
        mission.prepare_navigation(remap_ready=True)
        wait_for_new_pointcloud(planner, previous_revision, fc, navi, stop_event)

        from fleet_bus.rescue_task import attach_rescue_fleet_node

        fleet_node, task_receiver = attach_rescue_fleet_node(
            fc, navi, stop_event, allow_start=True, payload=payload,
            validate_plan=mission.validate_payload_ready,
            preflight_result=preflight_result)
        logger.warning("[GROUND-CHAIN] FAST-LIO/雷达/相机就绪；现在可启动小车。"
                       "本测试收到 START 后停在解锁前")
        task = wait_for_car_start(task_receiver, fc, stop_event, args.start_timeout)
        # wait_for_start 在应答写出前即被唤醒；给 FleetBus 工作线程发送 START ACK 的时间。
        ack_deadline = time.monotonic() + 0.5
        while time.monotonic() < ack_deadline:
            require_locked(fc, stop_event)
            stop_event.wait(min(0.1, ack_deadline - time.monotonic()))
        if fleet_node.write_failures:
            raise RuntimeError("FleetBus ACK write failed during ground test")
        allocation = validate_allocation(task.red, task.blue, task.green)
        mission.ledger = DropLedger(allocation)
        mission.prepare_navigation(remap_ready=task_receiver.remap_state == "ready")
        mission._check(check_deadline=False)
        require_locked(fc, stop_event)
        if not planner.get_debug_state()["ready"]:
            raise RuntimeError("radar point cloud became stale before ground test completed")
        logger.warning("[GROUND-CHAIN] PASS: car task={} allocation={} "
                       "CLOSE/REMAP/START accepted; FC remains locked; no flight or recognition",
                       task.task_id, allocation)
        return 0
    except KeyboardInterrupt:
        logger.warning("[GROUND-CHAIN] Interrupted by user")
        return 130
    except Exception:
        logger.exception("[GROUND-CHAIN] Failed before unlock")
        return 1
    finally:
        stop_event.set()
        if task_receiver is not None and not task_receiver.join_remap(timeout=35.0):
            logger.error("[GROUND-CHAIN] Remap cancellation pending")
        if fleet_node is not None:
            try:
                fleet_node.close()
            except Exception:
                logger.exception("[GROUND-CHAIN] Failed to close car task receiver")
        if mission is not None:
            try:
                mission.stop()
            except Exception:
                logger.exception("[GROUND-CHAIN] Failed to stop navigation/camera")
        else:
            if navi is not None:
                try:
                    navi.stop()
                except Exception:
                    logger.exception("[GROUND-CHAIN] Failed to stop navigation")
            if vision is not None:
                try:
                    vision.close()
                except Exception:
                    logger.exception("[GROUND-CHAIN] Failed to close camera")
        if payload is not None:
            try:
                payload.close()
            except Exception:
                logger.exception("[GROUND-CHAIN] Failed to close STM32 link")
        if navi is not None:
            try:
                stop_navigation_ros(navi)
            except Exception:
                logger.exception("[GROUND-CHAIN] Failed to stop LIO ROS executor")
        if fc is not None:
            try:
                fc.close()
            except Exception:
                logger.exception("[GROUND-CHAIN] Failed to close FC_Server connection")
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    sys.exit(main())
