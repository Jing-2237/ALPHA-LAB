"""Wheel control, waypoint tracking, and a latched simulation safety gate."""

from dataclasses import dataclass
import math

from .model import Config, Pose, wrap_angle


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class PID:
    """PID with conditional-integration anti-windup and explicit reset."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, target: float, measured: float, dt: float, feedforward: float = 0.0) -> float:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if not all(math.isfinite(v) for v in (target, measured, feedforward)):
            raise ValueError("PID inputs must be finite")
        error = target - measured
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        candidate = self.integral + error * dt
        output = feedforward + self.kp * error + self.ki * candidate + self.kd * derivative
        if abs(output) <= 1 or output * error < 0:
            self.integral = candidate
        self.previous_error = error
        return clamp(feedforward + self.kp * error + self.ki * self.integral + self.kd * derivative, 1.0)


class WaypointFollower:
    def __init__(self, waypoints: list[tuple[float, float]], cfg: Config):
        if not waypoints or any(len(p) != 2 or not all(math.isfinite(x) for x in p) for p in waypoints):
            raise ValueError("waypoints must contain finite x/y pairs")
        self.waypoints = list(waypoints)
        self.cfg = cfg
        self.index = 0

    @property
    def done(self) -> bool:
        return self.index == len(self.waypoints)

    def command(self, pose: Pose) -> tuple[float, float]:
        while not self.done:
            x, y = self.waypoints[self.index]
            dx, dy = x - pose.x, y - pose.y
            distance = math.hypot(dx, dy)
            if distance <= self.cfg.goal_tolerance:
                self.index += 1
                continue
            error = wrap_angle(math.atan2(dy, dx) - pose.yaw)
            v = min(self.cfg.max_linear_speed, 1.2 * distance) * max(0.0, math.cos(error))
            w = clamp(2.8 * error, self.cfg.max_angular_speed)
            return v, w
        return 0.0, 0.0


@dataclass
class SafetyGate:
    """Latched stop; reconstruct only after the fault has been investigated."""

    cfg: Config
    reason: str = ""

    def filter(self, command: tuple[float, float], now: float, stamp: float,
               clearance: float, emergency: bool = False) -> tuple[float, float]:
        if not self.reason:
            if emergency:
                self.reason = "emergency_stop"
            elif not all(math.isfinite(v) for v in (*command, now, stamp)):
                self.reason = "invalid_command"
            elif now < stamp or now - stamp > self.cfg.command_timeout:
                self.reason = "command_timeout"
            elif math.isnan(clearance) or clearance < self.cfg.stop_distance:
                self.reason = "obstacle"
        if self.reason:
            return 0.0, 0.0
        return clamp(command[0], self.cfg.max_linear_speed), clamp(command[1], self.cfg.max_angular_speed)
