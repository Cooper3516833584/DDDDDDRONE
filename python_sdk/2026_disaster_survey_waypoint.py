"""
2026 模拟赛 — 逐航点阻塞导航版空地协同测绘救灾任务。

相机采集和地形模型在任务期间持续运行；主飞行线程逐个阻塞导航到测绘
航点，只在确认到达后读取到点之后的新视觉结果并记录地形标签。
"""
import importlib.util
import os
import sys
import threading
import time
from types import ModuleType
from typing import List, Optional, Tuple

from loguru import logger


def _load_base_task() -> ModuleType:
    module_name = "_disaster_survey_base"
    module_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "2026_disaster_survey.py",
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load base task from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


BASE_TASK = _load_base_task()

WAYPOINT_VISION_TIMEOUT = 3.0  # s
WAYPOINT_VISION_POLL_INTERVAL = 0.02  # s
WAYPOINT_VISION_MIN_DURATION = 1.0  # s; keeps the blue measurement LED visible
WAYPOINT_VISION_MAX_DISTANCE_PX = 150.0
WAYPOINT_BLUE_PREVIEW_SECONDS = 1.0
INDICATOR_BLUE = (0, 0, 255)
INDICATOR_RED = (255, 0, 0)


class WaypointMission(BASE_TASK.Mission):
    """逐航点阻塞导航；到点后读取新视觉结果并执行对应动作。"""

    def _read_waypoint_label(
        self,
        waypoint_index: int,
        raw_waypoint: Tuple[float, float],
    ) -> Tuple[Optional[str], int]:
        if self.sim_vision is None:
            logger.error("[SURVEY] sim_vision unavailable at waypoint")
            return None, 0

        after_seq = self.sim_vision.latest_frame_seq
        observations: List[BASE_TASK.RingObservation] = []
        started_at = time.perf_counter()
        earliest_finish = started_at + WAYPOINT_VISION_MIN_DURATION
        deadline = started_at + WAYPOINT_VISION_TIMEOUT
        rejected_off_center = 0

        self._tvlog(
            f"WP {waypoint_index} ARRIVED "
            f"({raw_waypoint[0]:.0f}, {raw_waypoint[1]:.0f}) "
            f"after_frame={after_seq}"
        )

        while time.perf_counter() < deadline:
            if self._stop_ring.is_set():
                raise RuntimeError("Mission stopped while reading waypoint vision")
            if self._ring_failed.is_set():
                raise RuntimeError("Terrain-ring detector stopped unexpectedly")

            observation = self._get_latest_ring_observation()
            if observation is not None and observation.frame_seq > after_seq:
                after_seq = observation.frame_seq
                distance_px = (
                    observation.offset_x ** 2 + observation.offset_y ** 2
                ) ** 0.5
                if (
                    observation.label in BASE_TASK.TERRAIN_LABEL_TO_CODE
                    and distance_px < WAYPOINT_VISION_MAX_DISTANCE_PX
                ):
                    observations.append(observation)
                elif observation.label in BASE_TASK.TERRAIN_LABEL_TO_CODE:
                    rejected_off_center += 1

                selected_label = self._select_survey_label(observations)
                if (
                    selected_label is not None
                    and time.perf_counter() >= earliest_finish
                ):
                    self._tvlog(
                        f"WP {waypoint_index} LABEL "
                        f"samples={len(observations)} "
                        f"rejected_off_center={rejected_off_center} "
                        f"consensus={selected_label}"
                    )
                    return selected_label, len(observations)

            time.sleep(WAYPOINT_VISION_POLL_INTERVAL)

        selected_label = self._select_survey_label(observations)
        self._tvlog(
            f"WP {waypoint_index} LABEL_TIMEOUT "
            f"samples={len(observations)} "
            f"rejected_off_center={rejected_off_center} "
            f"consensus={selected_label}"
        )
        return selected_label, len(observations)

    def _handle_waypoint_label(
        self,
        waypoint_index: int,
        raw_waypoint: Tuple[float, float],
        label: Optional[str],
        sample_count: int,
    ) -> None:
        if label is None:
            logger.warning(
                f"[SURVEY] WP {waypoint_index} has no stable terrain label"
            )
            return

        self._record_survey_label(
            raw_waypoint[0],
            raw_waypoint[1],
            label=label,
            sample_count=sample_count,
            flash_wildfire=False,
        )

        if label == "wildfire":
            self.fc.set_indicator_led(*INDICATOR_RED)
            logger.info(f"[SURVEY] WP {waypoint_index} wildfire -> red indicator")
        elif label == "debris_flow" and not self._debris_flow_triggered_once:
            self._debris_flow_triggered_once = True
            self._debris_flow_wp_index = waypoint_index
            logger.info(
                f"[SURVEY] WP {waypoint_index} debris_flow -> disaster action"
            )
            self._action_debris_flow()

    def run(self) -> None:
        fc = self.fc
        navi = self.navi

        raw_waypoints: List[Tuple[float, float]] = [
            (100, -40),
            (100, -110),
            (100, -180),
            (100, -250),
            (100, -320),
            (170, -320),
            (170, -250),
            (170, -180),
            (170, -110),
            (170, -40),
            (240, -40),
            (240, -110),
            (240, -180),
            (240, -250),
            (240, -320),
        ]
        landing_wp = (0.0, 0.0)

        if (
            self.sim_vision is not None
            and not self.sim_vision.warm_up_ring_detector()
        ):
            raise RuntimeError("Terrain-ring detector warm-up failed")

        navi.set_navigation_speed(BASE_TASK.CRUISE_SPEED)
        navi.set_vertical_speed(BASE_TASK.VERTICAL_SPEED)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        logger.info("[MISSION] Navigation started (fusion-ros)")
        navi.set_rs_speed_report(True, 2)

        fc.set_action_log(False)
        fc.set_indicator_led(0, 255, 0)
        fc.set_action_log(True)
        logger.info("[MISSION] Waypoint mission started")

        cart_timeout = 30.0
        logger.info(f"[MISSION] Waiting Cartographer TF ({cart_timeout}s)...")
        started_at = time.perf_counter()
        while True:
            time.sleep(1)
            logger.info(f"[MISSION] current_point: {navi.current_point}")
            if navi.current_point[0] + navi.current_point[1] != 0:
                break
            if time.perf_counter() - started_at > cart_timeout:
                raise RuntimeError(f"Cartographer TF timeout ({cart_timeout}s)")
        logger.info(
            f"[MISSION] Cartographer TF ok "
            f"({time.perf_counter() - started_at:.1f}s)"
        )
        fc.set_indicator_led(0, 0, 0)

        logger.info(f"[MISSION] Takeoff to {self.cruise_height}cm")
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        time.sleep(0.5)

        if self.sim_vision is not None:
            if not self._calibrate_to_takeoff_rectangle():
                raise RuntimeError("Takeoff calibration failed")
        else:
            raise RuntimeError("sim_vision unavailable")

        self._origin_x = float(navi.current_x)
        self._origin_y = float(navi.current_y)
        logger.info(
            f"[MISSION] Origin = ({self._origin_x:.1f}, {self._origin_y:.1f})"
        )

        if not self._start_ring_detection():
            raise RuntimeError("Terrain-ring detector did not become ready")

        # 保持相机和模型持续运行，但禁用巡航途中的泥石流自动触发。
        self._ring_actions_enabled.clear()

        for waypoint_index, raw_waypoint in enumerate(raw_waypoints):
            abs_waypoint = (
                raw_waypoint[0] + self._origin_x,
                raw_waypoint[1] + self._origin_y,
            )
            logger.info(
                f"[MISSION] Navigating to WP {waypoint_index} "
                f"raw={raw_waypoint}, absolute={abs_waypoint}"
            )
            reached = navi.navigation_to_waypoint(abs_waypoint, wait=True)
            if not reached:
                navi.stop_move()
                raise RuntimeError(
                    f"Navigation failed at WP {waypoint_index} "
                    f"{raw_waypoint}"
                )

            fc.set_indicator_led(*INDICATOR_BLUE)
            logger.info(
                f"[MISSION] WP {waypoint_index} measurement started "
                f"-> blue indicator"
            )
            try:
                if self._stop_ring.wait(WAYPOINT_BLUE_PREVIEW_SECONDS):
                    raise RuntimeError(
                        "Mission stopped while showing waypoint indicator"
                    )
                label, sample_count = self._read_waypoint_label(
                    waypoint_index,
                    raw_waypoint,
                )
            finally:
                fc.set_indicator_led(0, 0, 0)
                logger.info(
                    f"[MISSION] WP {waypoint_index} measurement finished "
                    f"-> indicator off"
                )
            self._handle_waypoint_label(
                waypoint_index,
                raw_waypoint,
                label,
                sample_count,
            )

        logger.info("=" * 50)
        logger.info("[MISSION] === Final Survey Grid ===")
        self._log_survey_grid()
        self._mark_survey_complete()
        logger.info("=" * 50)

        self._ring_actions_enabled.clear()
        self._stop_ring.set()

        logger.info("[MISSION] Landing at navigation origin")
        navi.pointing_landing(landing_wp)


def main() -> None:
    rm = BASE_TASK.RosManager()
    rm.chmod("/dev/ttyUSB0")
    rm.chmod("/dev/ttyACM0")
    rm.chmod("/dev/video0")

    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package(
        "tf2_ros",
        "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link",
    )

    fc = BASE_TASK.FC_Client()
    fc.connect()
    time.sleep(0.5)

    t265 = BASE_TASK.T265("ros")
    t265.start()
    radar = BASE_TASK.LD_Radar()
    radar.start("ros")
    screen = BASE_TASK.UARTScreen(fc)

    sim_vision = BASE_TASK.SimVisionTask(camera_index=BASE_TASK.CAMERA_INDEX)
    sim_vision.open()

    mapper = BASE_TASK.RosMapper()
    navi = BASE_TASK.Navigation(fc=fc, rs=t265, radar=radar, mapper=mapper)
    BASE_TASK.RosNodeRunner().add_nodes().run()

    mission = WaypointMission(
        fc=fc,
        rs=t265,
        radar=radar,
        navi=navi,
        sim_vision=sim_vision,
    )
    remote_stop_event = threading.Event()
    fleet_node = None

    def wait_for_remote_stop() -> None:
        remote_stop_event.wait()
        logger.warning("[FLEET] Remote STOP received")
        mission.stop()

    try:
        fleet_node = BASE_TASK.attach_air_fleet_node(
            fc,
            navi,
            remote_stop_event,
            readonly=True,
            survey_provider=mission.get_survey_state,
        )
        threading.Thread(
            target=wait_for_remote_stop,
            name="fleet-remote-stop",
            daemon=True,
        ).start()
        mission.run()
    except Exception as exc:
        logger.exception(f"[MANAGER] Mission Failed: {exc}")
    finally:
        if fleet_node is not None:
            try:
                fleet_node.close()
            except Exception as exc:
                logger.exception(f"[FLEET] Close failed: {exc}")
        mission.stop()
        if fc.state.unlock.value:
            logger.warning("[MANAGER] Auto Landing (Emergency)")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            locked = fc.wait_for_lock()
            if not locked:
                fc.lock()
        sim_vision.close()

    logger.info("[MANAGER] Mission finished")
    fc.close()


if __name__ == "__main__":
    main()
