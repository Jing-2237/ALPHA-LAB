"""Deterministic closed-loop simulation; no hardware or physics-engine dependency."""

from dataclasses import dataclass
import math

from .control import PID, SafetyGate, WaypointFollower
from .model import Config, Pose, integrate, wheel_targets

DEFAULT_ROUTE = [(1.5, 0.0), (1.5, 1.0), (0.0, 1.0), (0.0, 0.0)]


@dataclass
class SimulationResult:
    status: str
    reason: str
    rows: list[dict]
    completed_waypoints: int
    final_error: float


def simulate(cfg: Config, waypoints=None, duration: float = 60.0,
             obstacle_at: float | None = None) -> SimulationResult:
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    if obstacle_at is not None and (not math.isfinite(obstacle_at) or obstacle_at < 0):
        raise ValueError("obstacle_at must be finite and nonnegative")
    route = DEFAULT_ROUTE if waypoints is None else waypoints
    nav, gate = WaypointFollower(route, cfg), SafetyGate(cfg)
    controllers = [PID(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd) for _ in range(2)]
    pose, wheels, rows = Pose(), [0.0, 0.0], []
    status = "timeout"
    for step in range(math.ceil(duration / cfg.dt)):
        now = step * cfg.dt
        command = nav.command(pose)
        clearance = 0.1 if obstacle_at is not None and now >= obstacle_at else math.inf
        v, w = gate.filter(command, now, now, clearance)
        targets = wheel_targets(v, w, cfg)
        efforts = [pid.update(target, actual, cfg.dt, target / cfg.max_wheel_speed)
                   for pid, target, actual in zip(controllers, targets, wheels)]
        if gate.reason or nav.done:
            # Explicit ideal brake model, not a prediction of real stopping distance.
            wheels, efforts = [0.0, 0.0], [0.0, 0.0]
            for pid in controllers:
                pid.reset()
        else:
            gain = 1 - math.exp(-cfg.dt / cfg.motor_time_constant)
            wheels = [actual + gain * (effort * cfg.max_wheel_speed - actual)
                      for actual, effort in zip(wheels, efforts)]
        pose = integrate(pose, *wheels, cfg.dt, cfg)
        rows.append(dict(time_s=(step + 1) * cfg.dt, x_m=pose.x, y_m=pose.y, yaw_rad=pose.yaw,
                         v_cmd_m_s=v, w_cmd_rad_s=w, left_rad_s=wheels[0], right_rad_s=wheels[1],
                         left_effort=efforts[0], right_effort=efforts[1], waypoint=nav.index,
                         safety=gate.reason or "running"))
        if gate.reason or nav.done:
            status = "stopped" if gate.reason else "completed"
            break
    goal = route[-1]
    return SimulationResult(status, gate.reason, rows, nav.index, math.hypot(pose.x - goal[0], pose.y - goal[1]))
