"""救援投放实飞测试入口：红、蓝、绿各一件，同一黄色目标两件。

沿用 rescue_drop_2026.py 的航迹、视觉、避障、继电器和安全退出流程。
真实飞行仍需 --confirm-flight、三个颜色配额参数和终端 start_mission。
"""

import sys

import rescue_drop_2026 as rescue


FREE_DROP_COUNT = 3
MANDATORY_DROP_COUNT = 2
TOTAL_DROP_COUNT = FREE_DROP_COUNT + MANDATORY_DROP_COUNT


def validate_test_allocation(red: int, blue: int, green: int):
    if any(type(count) is not int or count != 1 for count in (red, blue, green)):
        raise ValueError("this test requires --red-count 1 --blue-count 1 --green-count 1")
    return {"red": red, "blue": blue, "green": green}


class TwoYellowMission(rescue.Mission):
    def _mandatory_drop(self) -> bool:
        target = self.target
        if target is None or target.color != rescue.MANDATORY_COLOR:
            raise RuntimeError("mandatory target missing")
        approach_observation = self._approach_target(target, protected=True)
        if approach_observation is None:
            self._resume_center_route()
            return False
        self._check()
        if self.obstacle is None:
            raise RuntimeError("obstacle interface missing")
        if not self.ledger.has_quota(rescue.MANDATORY_COLOR):
            raise RuntimeError("mandatory quota exhausted")

        target_world = self._estimate_target_world_xy(approach_observation)
        next_drop = self.ledger.next_drop_number
        last_drop = next_drop + self.ledger.mandatory_remaining - 1
        if last_drop > rescue.TOTAL_DROP_COUNT:
            raise RuntimeError("planned relay channels exhausted")
        # 两件都将移到各自的悬挂点上方；下降前分别确认目标姿态净空。
        for drop_number in range(next_drop, last_drop + 1):
            desired_pose = self._desired_aircraft_drop_pose(target_world, drop_number)
            if not self.obstacle.mandatory_drop_pose_is_clear(desired_pose):
                rescue.logger.warning(
                    "[MANDATORY] 43cm drop-pose clearance rejected: "
                    "target={} drop={} target_world={} pose={}",
                    target.target_id, drop_number, target_world, desired_pose)
                self.navi.stop_move()
                self._mark_mandatory_pose_rejected(target.target_id)
                self._resume_center_route()
                return False
            rescue.logger.info(
                "[MANDATORY] 43cm drop-pose clearance accepted: "
                "target={} drop={} target_world={} pose={}",
                target.target_id, drop_number, target_world, desired_pose)

        self._set_height(rescue.MANDATORY_DROP_HEIGHT)
        while self.ledger.has_quota(rescue.MANDATORY_COLOR):
            self._check()
            drop_number = self.ledger.next_drop_number
            calibrated = self._calibrate_low(
                target, protected=True, drop_number=drop_number)
            self._drop(rescue.MANDATORY_COLOR, target.target_id, calibrated)
        self._set_height(rescue.CRUISE_HEIGHT)
        self._navigate_center_exit()
        return True


def main() -> int:
    # 仅在本测试入口进程中覆盖投放配额；主入口文件及其默认行为不变。
    rescue.FREE_DROP_COUNT = FREE_DROP_COUNT
    rescue.MANDATORY_DROP_COUNT = MANDATORY_DROP_COUNT
    rescue.TOTAL_DROP_COUNT = TOTAL_DROP_COUNT
    rescue.validate_allocation = validate_test_allocation
    rescue.Mission = TwoYellowMission
    return rescue.main()


if __name__ == "__main__":
    sys.exit(main())
