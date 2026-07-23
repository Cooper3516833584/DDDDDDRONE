"""2024 D problem, requirement 2: targeted inventory mission.

Flow:
  ground SSH ``start`` -> START_VISION_ACQUIRE over the FC UT2/HC-14 bridge
  -> scan front camera index 0 without a timeout
  -> look up the detected cargo in 2024_D_24_inventory.json
  -> report the cargo/location and count down for 10 seconds
  -> take off, fly around the north (+Y) end of the shelves
  -> approach the requested face from north to south, verify the QR and flash
     the laser/ground LED
  -> leave in +Y and land on the black circle.

During the unlimited preflight scan the aircraft link remains in command RX
mode, so a ground ``quit``/STOP_MISSION can cancel it immediately. Once the
aircraft begins telemetry transmission and flight, GroundStationLink continues
through the flight controller bridge in telemetry mode and deliberately ignores
in-flight commands.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import time
from typing import Optional

import numpy as np
from loguru import logger

from FlightController import FC_Client, FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosManager import RosManager
from FlightController.Components.GroundStationLink import (
    CommandId,
    MissionState,
    RejectReason,
)

from task2_route_plan import TargetRoute, load_inventory_map, route_for_location


ROOT = Path(__file__).resolve().parent
REFERENCE_PATH = ROOT / "2024_D_24.py"
INVENTORY_PATH = ROOT / "2024_D_24_inventory.json"

CRUISE_SPEED = 22
CRUISE_HEIGHT = 150.0
VERTICAL_SPEED = 22
FRONT_QR_CAMERA_INDEX = 0
LANDING_CAMERA_INDEX = 2
COUNTDOWN_SECONDS = 10
STABLE_DETECTION_FRAMES = 3
GROUND_LED_WHITE = ((255, 255, 255),) * 7
GROUND_LED_OFF = ((0, 0, 0),) * 7


def _load_reference_module():
    spec = importlib.util.spec_from_file_location("reference_2024_D_24", REFERENCE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load reference mission: {REFERENCE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REFERENCE = _load_reference_module()


class TargetInventoryMission(REFERENCE.Mission):
    """Target-only mission reusing the proven vision/radar/landing primitives."""

    def __init__(
        self,
        *,
        fc: FC_Like,
        radar: LD_Radar,
        navi: Navigation,
        rs: T265,
        mapper: Optional[RosMapper] = None,
    ):
        # Do not call the task-1 constructor: it initializes an empty inventory
        # and would overwrite 2024_D_24_inventory.json.
        self.fc = fc
        self.radar = radar
        self.navi = navi
        self.rs = rs
        self.mapper = mapper
        self.cruise_height = CRUISE_HEIGHT
        self.qr_camera_index = FRONT_QR_CAMERA_INDEX
        self.landing_camera_index = LANDING_CAMERA_INDEX
        self._last_qr_offset = None
        self._qr_round = 0
        self._qr_positions = {}
        self._inventory_record_path = str(INVENTORY_PATH)

    def scan_preflight_target(self) -> Optional[int]:
        """Scan camera 0 until a stable 1..24 result or ground STOP."""

        cap = REFERENCE._open_usb_camera(self.qr_camera_index, 1280, 720)
        last_number: Optional[int] = None
        stable_count = 0
        failed_reads = 0
        last_log = 0.0
        try:
            for _ in range(5):
                cap.read()
            logger.info("[TASK2] Camera 0 target acquisition started (no timeout)")
            while not self.fc.ground_stop_requested:
                ok, frame = cap.read()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 100:
                        raise RuntimeError("camera 0 did not return frames")
                    time.sleep(0.05)
                    continue
                failed_reads = 0
                detections = REFERENCE._detect_qrcodes(frame)
                selected = None
                if detections:
                    image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                    selected = REFERENCE._select_nearest_to_image_center(
                        detections, image_center
                    )
                try:
                    number = int(selected.text.strip()) if selected is not None else 0
                except (AttributeError, TypeError, ValueError):
                    number = 0
                if not 1 <= number <= 24:
                    last_number = None
                    stable_count = 0
                elif number == last_number:
                    stable_count += 1
                else:
                    last_number = number
                    stable_count = 1
                if stable_count >= STABLE_DETECTION_FRAMES:
                    logger.info(f"[TASK2] Stable preflight QR detected: cargo #{number}")
                    return number
                now = time.monotonic()
                if now - last_log >= 5.0:
                    logger.info("[TASK2] Waiting for QR 1..24 in camera 0; type quit to cancel")
                    last_log = now
                time.sleep(0.03)
            logger.info("[TASK2] Preflight QR acquisition cancelled by ground STOP")
            return None
        finally:
            cap.release()

    def countdown(self, cargo_number: int, route: TargetRoute) -> None:
        for remaining in range(COUNTDOWN_SECONDS, 0, -1):
            self.fc.send_ground_status(
                MissionState.COUNTDOWN,
                target1=cargo_number,
                target2=route.location_ordinal,
                progress=0,
                message=f"TGT:COUNTDOWN:{remaining}:{cargo_number}:{route.location}",
            )
            logger.info(f"[TASK2] Takeoff in {remaining}s")
            time.sleep(1.0)

    def _wait_for_cartographer(self, timeout: float = 30.0) -> None:
        started = time.perf_counter()
        while True:
            time.sleep(1.0)
            if self.navi.current_point[0] + self.navi.current_point[1] != 0:
                logger.info("[TASK2] Cartographer TF established")
                return
            if time.perf_counter() - started > timeout:
                raise RuntimeError("Cartographer TF was not established after takeoff")

    def _navigate_local(self, origin: np.ndarray, point, label: str) -> None:
        target = origin + np.asarray(point, dtype=float)
        logger.info(f"[TASK2] {label}: local={point}, map={np.round(target, 1)}")
        self.navi.navigation_to_waypoint(target, wait=True)

    def _verify_and_indicate(
        self,
        cargo_number: int,
        route: TargetRoute,
    ) -> None:
        navi = self.navi
        navi.set_height(route.scan_height_cm)
        navi.wait_for_height()
        if abs(navi.current_height - route.scan_height_cm) >= 8.0:
            raise RuntimeError("target scan height was not reached")
        if not self.barrier_distance_align():
            raise RuntimeError("shelf distance alignment failed")
        if not self.vision_qr_approach(timeout=40.0):
            raise RuntimeError("target QR visual alignment failed")

        verified = self._detect_qr_number_and_offset()
        actual_number = verified[0] if verified is not None else 0
        if actual_number != cargo_number:
            raise RuntimeError(
                f"target QR mismatch: expected {cargo_number}, got {actual_number}"
            )

        self.fc.send_ground_status(
            MissionState.RUNNING,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=65,
            message=f"TGT:VERIFIED:{cargo_number}:{route.location}",
        )
        self.fc.set_ground_led_pixels(GROUND_LED_WHITE, brightness=4)
        laser_started = False
        try:
            self.laser_on()
            laser_started = True
            time.sleep(0.5)
        finally:
            try:
                if laser_started:
                    self.laser_off()
            finally:
                time.sleep(0.5)
                self.fc.set_ground_led_pixels(GROUND_LED_OFF, brightness=0)

    def run_target(self, cargo_number: int, route: TargetRoute) -> None:
        fc = self.fc
        navi = self.navi
        navi.set_navigation_speed(CRUISE_SPEED)
        navi.set_vertical_speed(VERTICAL_SPEED)
        navi.start()
        navi.switch_navigation_mode("fusion-ros")
        fc.set_action_log(False)
        fc.set_action_log(True)

        self.fc.send_ground_status(
            MissionState.RUNNING,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=5,
            message=f"TGT:TAKEOFF:{cargo_number}:{route.location}",
        )
        navi.pointing_takeoff((0, 0), self.cruise_height)
        navi.set_yaw(0)
        navi.wait_for_yaw()
        self._wait_for_cartographer()
        origin = np.asarray(navi.current_point, dtype=float).copy()

        self._navigate_local(origin, route.outbound_local[1], "fly Y+ to north corridor")
        self._navigate_local(origin, route.outbound_local[2], "cross north corridor")
        navi.set_yaw(route.face_yaw_deg)
        navi.wait_for_yaw()
        self._navigate_local(origin, route.outbound_local[3], "fly Y- to target QR")
        self.fc.send_ground_status(
            MissionState.RUNNING,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=45,
            message=f"TGT:ARRIVED:{cargo_number}:{route.location}",
        )
        self._verify_and_indicate(cargo_number, route)

        navi.set_height(self.cruise_height)
        navi.wait_for_height()
        self._navigate_local(origin, route.return_local[1], "leave target in Y+")
        navi.set_yaw(0)
        navi.wait_for_yaw()
        self._navigate_local(origin, route.return_local[2], "fly to landing point")

        self.fc.send_ground_status(
            MissionState.LANDING,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=90,
            message=f"TGT:LANDING:{cargo_number}:{route.location}",
        )
        navi.set_height(REFERENCE.LANDING_SCAN_HEIGHT)
        navi.wait_for_height()
        if not self.landing_vision_approach(timeout=45.0):
            raise RuntimeError("landing circle visual alignment failed")
        self.land_after_visual_alignment()


def _complete_stop_command(fc: FC_Client) -> None:
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        command = fc.receive_ground_command(timeout=0.2)
        if command is None:
            continue
        try:
            if command.command.command_id == CommandId.STOP_MISSION:
                fc.complete_ground_command(command)
                return
            fc.reject_ground_command(command, RejectReason.TASK_BUSY)
        finally:
            fc.ground_command_done()


def main() -> int:
    rm = RosManager()
    rm.chmod("/dev/ttyUSB0")  # CP2102 radar; GroundStationLink does not open it
    rm.chmod("/dev/ttyACM0")
    rm.chmod("/dev/video0")  # front QR camera required by task 2
    rm.chmod("/dev/video2")  # downward landing camera
    rm.launch_package("ldlidar_stl_ros2", "ld19.launch.py")
    rm.launch_package("realsense2_camera", "rs_launch.py")
    rm.launch_package("cartographer_ros", "cartographer.launch.py")
    rm.run_package(
        "tf2_ros",
        "static_transform_publisher",
        "0 0 0 0 0 0 camera_pose_frame base_link",
    )

    fc = FC_Client()
    fc.connect()
    time.sleep(0.5)
    t265 = T265("ros")
    t265.start()
    radar = LD_Radar()
    radar.start("ros")
    mapper = RosMapper()
    navi = Navigation(fc=fc, rs=t265, radar=radar, mapper=mapper)
    RosNodeRunner().add_nodes().run()
    mission = TargetInventoryMission(
        fc=fc,
        rs=t265,
        radar=radar,
        navi=navi,
        mapper=mapper,
    )

    start_command = None
    mission_error: Optional[Exception] = None
    try:
        # Reuse FC_Client through FCWirelessTransport -> FC 0x0D/0x07 -> UT2.
        # Do not open an airborne HC-14 USB serial device here.
        fc.start_ground_station()
        logger.info("[TASK2] GroundStationLink using FC wireless bridge (UT2)")
        fc.enable_ground_command_reception()
        logger.info("[TASK2] Waiting for SSH start / START_VISION_ACQUIRE")
        while start_command is None:
            command = fc.receive_ground_command(timeout=0.5)
            if command is None:
                continue
            try:
                if command.command.command_id in (
                    CommandId.START_VISION_ACQUIRE,
                    CommandId.START_MISSION,
                ):
                    fc.prepare_ground_mission()
                    fc.accept_ground_command(command)
                    start_command = command
                elif command.command.command_id == CommandId.STOP_MISSION:
                    fc.complete_ground_command(command)
                else:
                    fc.reject_ground_command(command, RejectReason.UNKNOWN_COMMAND)
            finally:
                fc.ground_command_done()

        cargo_number = mission.scan_preflight_target()
        if cargo_number is None:
            fc.fail_ground_command(start_command, RejectReason.TASK_BUSY)
            _complete_stop_command(fc)
            return 0
        inventory = load_inventory_map(INVENTORY_PATH)
        logger.info(f"[TASK2] Loaded {len(inventory)} task-1 inventory records")
        location = inventory.get(cargo_number)
        if location is None:
            logger.error(f"[TASK2] Cargo #{cargo_number} is absent from task-1 inventory")
            fc.fail_ground_command(start_command, RejectReason.BAD_TARGETS)
            return 2

        route = route_for_location(location)
        fc.enable_ground_telemetry()
        fc.send_ground_status(
            MissionState.READY,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=0,
            message=f"TGT:DETECTED:{cargo_number}:{route.location}",
        )
        mission.countdown(cargo_number, route)
        mission.run_target(cargo_number, route)
        fc.send_ground_status(
            MissionState.COMPLETED,
            target1=cargo_number,
            target2=route.location_ordinal,
            progress=100,
            message=f"TGT:COMPLETE:{cargo_number}:{route.location}",
        )
        fc.complete_ground_command(start_command)
        return 0
    except Exception as exc:
        mission_error = exc
        logger.exception(f"[TASK2] Mission failed: {exc}")
        if fc.ground_station is not None:
            try:
                fc.enable_ground_telemetry()
                fc.send_ground_status(
                    MissionState.FAILED,
                    progress=0,
                    error_code=1,
                    message=f"TGT:FAILED:{type(exc).__name__}",
                )
                if start_command is not None:
                    fc.fail_ground_command(start_command, RejectReason.FC_OFFLINE)
            except Exception:
                logger.exception("[TASK2] Failed to report mission error")
        return 1
    finally:
        try:
            mission.stop()
        except Exception:
            logger.exception("[TASK2] Navigation stop failed")
        if fc.state.unlock.value:
            logger.warning("[TASK2] Emergency landing")
            fc.set_flight_mode(fc.PROGRAM_MODE)
            fc.stablize()
            fc.land()
            if not fc.wait_for_lock():
                fc.lock()
        try:
            fc.set_ground_led_pixels(GROUND_LED_OFF, brightness=0)
        except Exception:
            if mission_error is None:
                logger.warning("[TASK2] Could not reset ground LED")
        fc.close()


if __name__ == "__main__":
    raise SystemExit(main())
