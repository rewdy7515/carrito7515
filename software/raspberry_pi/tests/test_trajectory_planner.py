"""Pruebas de geometría puras; no requieren cámara ni Arduino."""

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from trajectory_planner import (
    BypassSide,
    ObstaclePose,
    ObstaclePoseEstimator,
    SafeTrajectoryPlanner,
)


class SafeTrajectoryPlannerTests(unittest.TestCase):
    def test_green_left_plan_is_safe_when_obstacle_is_far(self) -> None:
        plan = SafeTrajectoryPlanner().plan_bypass(
            ObstaclePose(forward_mm=1600, lateral_mm=0),
            BypassSide.LEFT,
            road_wheel_angle_deg=35,
        )
        self.assertTrue(plan.safe)
        self.assertGreater(plan.target_lateral_mm, 0)
        self.assertEqual(plan.phases[1].steering, BypassSide.LEFT)

    def test_red_right_plan_is_safe_when_obstacle_is_far(self) -> None:
        plan = SafeTrajectoryPlanner().plan_bypass(
            ObstaclePose(forward_mm=1600, lateral_mm=0),
            BypassSide.RIGHT,
            road_wheel_angle_deg=35,
        )
        self.assertTrue(plan.safe)
        self.assertLess(plan.target_lateral_mm, 0)
        self.assertEqual(plan.phases[1].steering, BypassSide.RIGHT)

    def test_near_obstacle_is_rejected(self) -> None:
        plan = SafeTrajectoryPlanner().plan_bypass(
            ObstaclePose(forward_mm=100, lateral_mm=0),
            BypassSide.LEFT,
            road_wheel_angle_deg=35,
        )
        self.assertFalse(plan.safe)

    def test_bbox_is_converted_to_forward_distance_and_lateral_position(self) -> None:
        estimator = ObstaclePoseEstimator(focal_length_px=240)
        pose = estimator.estimate_from_bbox((130, 20, 20, 60), 320)
        self.assertAlmostEqual(pose.forward_mm, 400, delta=20)
        self.assertGreater(pose.lateral_mm, 0)


if __name__ == "__main__":
    unittest.main()
