"""移动目标视觉伴飞与同步下降的任务层控制组件。

该模块的纯逻辑配置和速度估计器可在无飞控、雷达、相机环境中导入。
硬件相关控制器只在实例化时加载现有视觉下降实现，不会在导入阶段
初始化设备、线程或任务。
"""

import threading
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np


MovingRecordCallback = Callable[
    [
        float,
        str,
        Optional[float],
        Optional[float],
        float,
        float,
        int,
        int,
    ],
    None,
]
VelocityPredictor = Callable[
    [Tuple[float, float], float],
    Tuple[float, float],
]
TargetOffsetProvider = Callable[[float], Tuple[float, float]]
HorizontalCommandGuard = Callable[[int, int], Tuple[int, int]]


@dataclass(frozen=True)
class MovingTargetDescentConfig:
    estimator_integral_gain: float = 0.048
    estimator_deadband_px: float = 3.0
    estimator_speed_limit: float = 10.0
    estimator_max_sample_dt: float = 0.2

    correction_gain: float = 0.15
    correction_deadband_px: float = 3.0
    horizontal_command_limit: float = 15.0
    command_filter_alpha: float = 0.25

    control_period: float = 0.05
    vision_sample_stale_seconds: float = 0.35
    vision_loss_timeout: float = 1.0

    def __post_init__(self) -> None:
        values = np.asarray(
            [
                self.estimator_integral_gain,
                self.estimator_deadband_px,
                self.estimator_speed_limit,
                self.estimator_max_sample_dt,
                self.correction_gain,
                self.correction_deadband_px,
                self.horizontal_command_limit,
                self.command_filter_alpha,
                self.control_period,
                self.vision_sample_stale_seconds,
                self.vision_loss_timeout,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Moving-target descent parameters must be finite")
        if min(
            self.estimator_integral_gain,
            self.estimator_deadband_px,
            self.correction_gain,
            self.correction_deadband_px,
        ) < 0:
            raise ValueError("Moving-target gains and deadbands must be non-negative")
        if min(
            self.estimator_speed_limit,
            self.estimator_max_sample_dt,
            self.horizontal_command_limit,
            self.control_period,
            self.vision_sample_stale_seconds,
            self.vision_loss_timeout,
        ) <= 0:
            raise ValueError("Moving-target limits and timeouts must be positive")
        if not 0 < self.command_filter_alpha <= 1:
            raise ValueError("command_filter_alpha must be in (0, 1]")


class TargetVelocityEstimator:
    """以视觉位置误差积分在线估计目标的机体系水平速度。"""

    def __init__(
        self,
        integral_gain: float,
        deadband_px: float,
        speed_limit: float,
        max_sample_dt: float,
    ):
        values = np.asarray(
            [
                integral_gain,
                deadband_px,
                speed_limit,
                max_sample_dt,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("Velocity-estimator parameters must be finite")
        if integral_gain < 0 or deadband_px < 0:
            raise ValueError("Estimator gain and deadband must be non-negative")
        if speed_limit <= 0 or max_sample_dt <= 0:
            raise ValueError("Estimator limits must be positive")

        self.integral_gain = float(integral_gain)
        self.deadband_px = float(deadband_px)
        self.speed_limit = float(speed_limit)
        self.max_sample_dt = float(max_sample_dt)
        self._velocity = np.zeros(2, dtype=float)
        self._lock = threading.Lock()

    @staticmethod
    def _limit_norm(vector: np.ndarray, limit: float) -> np.ndarray:
        speed = float(np.linalg.norm(vector))
        if speed > limit:
            return vector * (limit / speed)
        return vector

    def reset(self, initial_velocity: Tuple[float, float]) -> None:
        initial = np.asarray(initial_velocity, dtype=float)
        if initial.shape != (2,) or not np.all(np.isfinite(initial)):
            raise ValueError("initial_velocity must contain two finite values")
        initial = self._limit_norm(initial, self.speed_limit)
        with self._lock:
            self._velocity = initial.copy()

    def update(
        self,
        x_px: float,
        y_px: float,
        sample_dt: float,
        velocity_predictor: Optional[VelocityPredictor] = None,
    ) -> Tuple[float, float]:
        values = np.asarray([x_px, y_px, sample_dt], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("Velocity-estimator inputs must be finite")
        if velocity_predictor is not None and not callable(
            velocity_predictor
        ):
            raise ValueError("velocity_predictor must be callable")

        dt = min(max(float(sample_dt), 0.0), self.max_sample_dt)
        error = np.asarray([x_px, y_px], dtype=float)
        error[np.abs(error) <= self.deadband_px] = 0.0
        with self._lock:
            predicted = self._velocity.copy()
            if velocity_predictor is not None:
                predicted = np.asarray(
                    velocity_predictor(
                        (
                            float(predicted[0]),
                            float(predicted[1]),
                        ),
                        dt,
                    ),
                    dtype=float,
                )
                if (
                    predicted.shape != (2,)
                    or not np.all(np.isfinite(predicted))
                ):
                    raise ValueError(
                        "velocity_predictor must return two finite values"
                    )
                predicted = self._limit_norm(
                    predicted,
                    self.speed_limit,
                )
            updated = predicted + error * self.integral_gain * dt
            self._velocity = self._limit_norm(updated, self.speed_limit)
            return (
                float(self._velocity[0]),
                float(self._velocity[1]),
            )

    @property
    def velocity(self) -> Tuple[float, float]:
        with self._lock:
            return (
                float(self._velocity[0]),
                float(self._velocity[1]),
            )


class MovingTargetDescentController:
    """复用视觉下降控制器完成连续伴飞、同步下降和低空伴飞。"""

    def __init__(
        self,
        fc,
        navi,
        stop_event,
        latest_vision_sample: Callable,
        raise_if_vision_failed: Callable[[], None],
        record_callback: Optional[MovingRecordCallback] = None,
        config: Optional[MovingTargetDescentConfig] = None,
    ):
        from visual_target_descent import VisualTargetDescentController

        self.config = (
            MovingTargetDescentConfig()
            if config is None
            else config
        )
        if not isinstance(self.config, MovingTargetDescentConfig):
            raise ValueError("config must be MovingTargetDescentConfig")
        self._record_callback = record_callback
        self._estimator = TargetVelocityEstimator(
            integral_gain=self.config.estimator_integral_gain,
            deadband_px=self.config.estimator_deadband_px,
            speed_limit=self.config.estimator_speed_limit,
            max_sample_dt=self.config.estimator_max_sample_dt,
        )
        self._descent = VisualTargetDescentController(
            fc=fc,
            navi=navi,
            stop_event=stop_event,
            latest_vision_sample=latest_vision_sample,
            raise_if_vision_failed=raise_if_vision_failed,
            record_callback=self._record_adapter,
            correction_gain=self.config.correction_gain,
            correction_deadband_px=self.config.correction_deadband_px,
            horizontal_speed_limit=self.config.horizontal_command_limit,
            filter_alpha=self.config.command_filter_alpha,
            control_period=self.config.control_period,
            vision_sample_stale_seconds=(
                self.config.vision_sample_stale_seconds
            ),
            vision_loss_timeout=self.config.vision_loss_timeout,
        )

    def _provide_base_velocity(
        self,
        x_px: float,
        y_px: float,
        sample_dt: float,
    ) -> Tuple[float, float]:
        return self._estimator.update(x_px, y_px, sample_dt)

    def _record_adapter(
        self,
        started_at: float,
        phase: str,
        x_px: Optional[float],
        y_px: Optional[float],
        command_vx: int,
        command_vy: int,
    ) -> None:
        if self._record_callback is None:
            return
        estimated_vx, estimated_vy = self._estimator.velocity
        self._record_callback(
            started_at,
            phase,
            x_px,
            y_px,
            estimated_vx,
            estimated_vy,
            command_vx,
            command_vy,
        )

    def follow_and_descend(
        self,
        target_height: float,
        stabilize_seconds: float = 3.0,
        stabilize_timeout: float = 20.0,
        hover_seconds: float = 2.0,
        initial_target_velocity: Tuple[float, float] = (7.2, 0.0),
        height_tolerance: float = 5.0,
        height_confirm_time: float = 0.4,
        descent_timeout: float = 15.0,
        on_descent_start: Optional[Callable[[], None]] = None,
        on_height_reached: Optional[Callable[[], None]] = None,
        pre_descent_gate: Optional[Callable[[], bool]] = None,
        pre_descent_max_error_px: Optional[float] = None,
        velocity_predictor: Optional[VelocityPredictor] = None,
        target_offset_provider: Optional[TargetOffsetProvider] = None,
        horizontal_command_guard: Optional[HorizontalCommandGuard] = None,
    ) -> Tuple[float, float]:
        """阻塞执行连续伴飞、同步下降和目标高度伴飞。"""
        if velocity_predictor is not None and not callable(
            velocity_predictor
        ):
            raise ValueError("velocity_predictor must be callable")
        self._estimator.reset(initial_target_velocity)

        def provide_base_velocity(
            x_px: float,
            y_px: float,
            sample_dt: float,
        ) -> Tuple[float, float]:
            return self._estimator.update(
                x_px,
                y_px,
                sample_dt,
                velocity_predictor=velocity_predictor,
            )

        self._descent.descend_to_height(
            target_height=target_height,
            hover_seconds=hover_seconds,
            base_velocity=initial_target_velocity,
            height_tolerance=height_tolerance,
            height_confirm_time=height_confirm_time,
            timeout=descent_timeout,
            on_height_reached=on_height_reached,
            base_velocity_provider=provide_base_velocity,
            pre_descent_follow_seconds=stabilize_seconds,
            pre_descent_follow_timeout=stabilize_timeout,
            on_descent_start=on_descent_start,
            pre_descent_gate=pre_descent_gate,
            pre_descent_max_error_px=pre_descent_max_error_px,
            target_offset_provider=target_offset_provider,
            horizontal_command_guard=horizontal_command_guard,
        )
        return self._estimator.velocity

    @property
    def estimated_target_velocity(self) -> Tuple[float, float]:
        return self._estimator.velocity
