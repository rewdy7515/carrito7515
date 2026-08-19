import math
import unittest

try:
    from autonomous_controller import AutonomousController
    from geometric_planner import (
        GeometricPlanner, MotionPrimitive, PlannerInput, PlannerState,
        PrimitiveType, TrackDirection, TrajectoryProfile, VehicleGeometry,
        VehicleState, VisibleObstacle, rectangle_polygon, vehicle_step,
    )
except ImportError:
    from simulator.autonomous_controller import AutonomousController
    from simulator.geometric_planner import (
        GeometricPlanner, MotionPrimitive, PlannerInput, PlannerState,
        PrimitiveType, TrackDirection, TrajectoryProfile, VehicleGeometry,
        VehicleState, VisibleObstacle, rectangle_polygon, vehicle_step,
    )


BOUNDARY = rectangle_polygon((0.0, 0.0, 300.0, 300.0))


class GeometricPlannerTests(unittest.TestCase):
    def test_geometry_comes_from_physical_measurements(self):
        geometry = VehicleGeometry()
        self.assertEqual((geometry.length_cm, geometry.width_cm, geometry.wheelbase_cm), (21.15, 17.2, 15.0))
        self.assertEqual((geometry.minimum_right_radius_cm, geometry.minimum_left_radius_cm), (32.2, 43.0))

    def test_ackermann_is_asymmetric(self):
        geometry = VehicleGeometry()
        left, right = geometry.wheel_angles_deg(geometry.max_right_steering_deg)
        self.assertEqual((left, right), (22.75, 27.97))
        left, right = geometry.wheel_angles_deg(-geometry.max_left_steering_deg)
        self.assertEqual((left, right), (-21.45, -17.58))

    def test_primitives_reject_invalid_steering_sign(self):
        with self.assertRaises(ValueError):
            MotionPrimitive(PrimitiveType.ARC_LEFT, 10.0, 5.0, 18.0)
        with self.assertRaises(ValueError):
            MotionPrimitive(PrimitiveType.STRAIGHT, 10.0, 2.0, 18.0)

    def test_clear_projection_continues_straight(self):
        planner = GeometricPlanner()
        data = PlannerInput(VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY)
        result = planner.plan(data)
        self.assertEqual(result.state, PlannerState.FOLLOW)
        self.assertEqual(result.best_candidate.candidate_id, "STRAIGHT")
        self.assertGreater(result.command.target_speed_cm_s, 0.0)
        self.assertEqual(result.command.steering_angle_deg, 0.0)

    def test_blocked_projection_generates_three_geometric_profiles(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("1", 75.0, 50.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
                                           TrackDirection.CLOCKWISE, 0.0))
        profiles = {candidate.profile for candidate in result.candidates}
        self.assertTrue({TrajectoryProfile.CONSERVATIVE, TrajectoryProfile.NOMINAL,
                         TrajectoryProfile.TIGHT}.issubset(profiles))
        self.assertTrue(all(candidate.trajectory_points for candidate in result.candidates))

    def test_swept_footprint_detects_obstacle_not_only_center(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("edge", 70.0, 58.0, 5.0, 5.0, "unknown")
        result = planner.plan(PlannerInput(VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY))
        straight = result.candidates[0]
        self.assertFalse(straight.safe)
        self.assertIn(straight.rejection_reason, {"collision", "clearance"})

    def test_vehicle_step_uses_direction_dependent_radius(self):
        geometry = VehicleGeometry(max_steering_rate_deg_s=1000.0)
        right = vehicle_step(VehicleState(50.0, 50.0, 0.0, 18.0),
                             planner_command(18.0, geometry.max_right_steering_deg), 0.1, geometry)
        left = vehicle_step(VehicleState(50.0, 50.0, 0.0, 18.0),
                            planner_command(18.0, -geometry.max_left_steering_deg), 0.1, geometry)
        self.assertGreater(right.heading_rad, 0.0)
        self.assertLess(left.heading_rad, 0.0)
        self.assertGreater(abs(right.heading_rad), abs(left.heading_rad))

    def test_controller_interface_is_planner_input_to_result(self):
        controller = AutonomousController()
        result = controller.plan(PlannerInput(VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY))
        self.assertEqual(result.command.steering_angle_deg, 0.0)


def planner_command(speed: float, steering: float):
    from simulator.geometric_planner import ControlCommand
    return ControlCommand(speed, steering)


if __name__ == "__main__":
    unittest.main()
