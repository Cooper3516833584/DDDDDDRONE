"""2026 救援投放：已确认全航段无障碍时使用的任务入口。

仅省去障碍点云规划；飞控、LIO 位姿、视觉、继电器及任务停止保护
均由 rescue_drop_2026.py 负责。此入口仍需 --confirm-flight 和
终端 start_mission 指令才会起飞。不得在未确认全航段净空时使用。
"""

import sys
from types import SimpleNamespace
from typing import Callable, Tuple

import rescue_drop_2026 as rescue


Point = Tuple[float, float]
TRACK_START_OFFSET_X_CM = 0.0  # x_1：起飞点与航迹起点重合
TRACK_LENGTH_X_CM = 200.0       # x_2：前后飞行距离
RIGHT_SPAN_Y_CM = 157.0         # y_r：右侧距离
LEFT_SPAN_Y_CM = 160.0          # y_l：左侧距离


class ClearAreaPlanner:
    """仅供已确认无障碍场地使用；保留 Navigation 所需的接口。"""

    def bind_pose_getter(self, getter: Callable) -> None:
        # Navigation 的定位就绪和新鲜度检查仍由 LIO 自身执行。
        pass

    def on_pointcloud(self, msg) -> None:
        # 此入口不使用障碍点云，ROS 订阅回调中不执行规划。
        pass

    def safe_waypoint(self, current: Point, goal: Point) -> Point:
        return goal

    def safe_velocity(self, current: Point, velocity: Point) -> Point:
        return velocity

    def plan_route_window(self, current: Point, candidates):
        waypoints = tuple(candidates[:3])
        if not waypoints:
            return None
        return SimpleNamespace(waypoints=waypoints,
                               anchors=tuple(enumerate(waypoints)),
                               skipped_offsets=(), revision=0)

    def path_is_free(self, points) -> bool:
        return bool(points)

    def mandatory_drop_pose_is_clear(self, pose: Point) -> bool:
        return True


class ClearAreaObstacleInterface(rescue.ObstacleInterface):
    """仅限已确认全航段无障碍时，报告没有膨胀区。"""

    def point_is_inflated(self, point: Point) -> bool:
        return False

    def boundary_crossing(self, start: Point, end: Point, entering: bool) -> Point:
        return end if entering else start


def main() -> int:
    # 独立进程中的任务配置；不改动通用主入口文件的默认参数。
    rescue.TRACK_START_OFFSET_X_CM = TRACK_START_OFFSET_X_CM
    rescue.TRACK_LENGTH_X_CM = TRACK_LENGTH_X_CM
    rescue.RIGHT_SPAN_Y_CM = RIGHT_SPAN_Y_CM
    rescue.LEFT_SPAN_Y_CM = LEFT_SPAN_Y_CM
    rescue.ObstaclePlanner2D = ClearAreaPlanner
    rescue.ObstacleInterface = ClearAreaObstacleInterface
    return rescue.main()


if __name__ == "__main__":
    sys.exit(main())
