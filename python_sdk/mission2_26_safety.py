"""任务二调试场地临时安全层。

删除该功能时，只需删除本文件，并移除 ``mission2_26.py`` 中对应的导入、
实例化和 ``horizontal_command_guard`` 参数。
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class EscortXBoundaryVelocityGuard:
    """超过场地 x 上界时，将最终伴飞 x 速度指令强制为零。"""

    max_x: float = 357.5

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.max_x)):
            raise ValueError("max_x must be finite")

    def is_active(self, current_x: float) -> bool:
        current_x = float(current_x)
        if not math.isfinite(current_x):
            raise ValueError("current_x must be finite")
        return current_x > float(self.max_x)

    def apply(
        self,
        current_x: float,
        velocity_x: int,
        velocity_y: int,
    ) -> Tuple[int, int]:
        velocity_x = int(velocity_x)
        velocity_y = int(velocity_y)
        if self.is_active(current_x):
            velocity_x = 0
        return velocity_x, velocity_y


__all__ = ["EscortXBoundaryVelocityGuard"]
