import math
import unittest

from alpha_lab.control import PID, SafetyGate
from alpha_lab.model import Config, Pose, integrate, wheel_targets
from alpha_lab.simulation import simulate


class RobotTests(unittest.TestCase):
    def test_straight_odometry(self):
        p = integrate(Pose(), 2, 2, 1, Config())
        self.assertAlmostEqual(p.x, .1)
        self.assertAlmostEqual(p.y, 0)
        self.assertAlmostEqual(p.yaw, 0)

    def test_exact_arc(self):
        cfg = Config()
        p = integrate(Pose(), 0, 6, 1, cfg)
        self.assertAlmostEqual(p.x, .15 * math.sin(1))
        self.assertAlmostEqual(p.y, .15 * (1 - math.cos(1)))
        self.assertAlmostEqual(p.yaw, 1)

    def test_curvature_preserving_limit(self):
        left, right = wheel_targets(10, 2, Config())
        self.assertLessEqual(max(abs(left), abs(right)), 20)
        self.assertAlmostEqual(left / right, 9.7 / 10.3)

    def test_pid_saturation_does_not_wind_up(self):
        pid = PID(1, 1, 0)
        for _ in range(1000):
            self.assertEqual(pid.update(100, 0, .02), 1)
        self.assertEqual(pid.integral, 0)
        self.assertEqual(pid.update(0, 0, .02), 0)
        with self.assertRaises(ValueError):
            pid.update(0, 0, 0)

    def test_watchdog_latches(self):
        gate = SafetyGate(Config())
        self.assertEqual(gate.filter((.2, 0), 1, 0, math.inf), (0, 0))
        self.assertEqual(gate.reason, "command_timeout")
        self.assertEqual(gate.filter((.2, 0), 1, 1, math.inf), (0, 0))

    def test_safety_faults(self):
        for kwargs, reason in [({"clearance": .1}, "obstacle"),
                               ({"emergency": True}, "emergency_stop"),
                               ({"command": (math.nan, 0)}, "invalid_command"),
                               ({"stamp": 2}, "command_timeout")]:
            gate = SafetyGate(Config())
            inputs = dict(command=(.2, 0), now=1, stamp=1, clearance=math.inf)
            inputs.update(kwargs)
            self.assertEqual(gate.filter(**inputs), (0, 0))
            self.assertEqual(gate.reason, reason)

    def test_rectangle_mission(self):
        result = simulate(Config())
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.completed_waypoints, 4)
        self.assertLessEqual(result.final_error, .05)
        self.assertTrue(all(abs(row["left_effort"]) <= 1 and abs(row["right_effort"]) <= 1 for row in result.rows))

    def test_obstacle_stops_motion(self):
        result = simulate(Config(), obstacle_at=2)
        self.assertEqual(result.status, "stopped")
        self.assertEqual(result.reason, "obstacle")
        self.assertEqual(result.rows[-1]["left_rad_s"], 0)
        self.assertEqual(result.rows[-1]["right_rad_s"], 0)
        self.assertEqual(result.rows[-1]["v_cmd_m_s"], 0)

    def test_timeout_and_input_validation(self):
        self.assertEqual(simulate(Config(), duration=.1).status, "timeout")
        for values in ({"wheel_radius": 0}, {"dt": math.nan}, {"track_width": -1}, {"dt": 1}):
            with self.assertRaises(ValueError):
                Config(**values)
        with self.assertRaises(ValueError):
            simulate(Config(), waypoints=[])


if __name__ == "__main__":
    unittest.main()
