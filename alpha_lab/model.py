"""SI-unit robot model and exact differential-drive integration."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Config:
    wheel_radius: float = 0.05
    track_width: float = 0.30
    max_linear_speed: float = 0.35
    max_angular_speed: float = 1.8
    max_wheel_speed: float = 20.0
    motor_time_constant: float = 0.12
    dt: float = 0.02
    goal_tolerance: float = 0.05
    command_timeout: float = 0.25
    stop_distance: float = 0.25
    pid_kp: float = 0.035
    pid_ki: float = 0.06
    pid_kd: float = 0.001

    def __post_init__(self):
        for key, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be numeric")
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
            if not key.startswith("pid_") and value == 0:
                raise ValueError(f"{key} must be positive")
        if self.dt > self.command_timeout:
            raise ValueError("dt must not exceed command_timeout")

    @classmethod
    def load(cls, path: str | Path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def wheel_targets(v: float, w: float, cfg: Config) -> tuple[float, float]:
    left = (v - w * cfg.track_width / 2) / cfg.wheel_radius
    right = (v + w * cfg.track_width / 2) / cfg.wheel_radius
    # Scale both wheels together to preserve curvature under saturation.
    scale = max(1.0, abs(left) / cfg.max_wheel_speed, abs(right) / cfg.max_wheel_speed)
    return left / scale, right / scale


def integrate(pose: Pose, left: float, right: float, dt: float, cfg: Config) -> Pose:
    v = cfg.wheel_radius * (left + right) / 2
    w = cfg.wheel_radius * (right - left) / cfg.track_width
    angle = w * dt
    if abs(w) < 1e-10:
        dx, dy = v * dt * math.cos(pose.yaw), v * dt * math.sin(pose.yaw)
    else:
        dx = v / w * (math.sin(pose.yaw + angle) - math.sin(pose.yaw))
        dy = v / w * (math.cos(pose.yaw) - math.cos(pose.yaw + angle))
    return Pose(pose.x + dx, pose.y + dy, wrap_angle(pose.yaw + angle))
