import math
import unittest

try:
    from planner_test_runner import SensorModel, run_scenario
except ImportError:
    from simulator.planner_test_runner import SensorModel, run_scenario

try:
    from autonomous_controller import AutonomousController
    from geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, PlannerInput, PlannerState,
        PrimitiveType, TrackDirection, TrajectoryProfile, VehicleGeometry,
        TrajectoryPoint, VehicleState, VisibleObstacle, rectangle_polygon, vehicle_step,
    )
except ImportError:
    from simulator.autonomous_controller import AutonomousController
    from simulator.geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, PlannerInput, PlannerState,
        PrimitiveType, TrackDirection, TrajectoryProfile, VehicleGeometry,
        TrajectoryPoint, VehicleState, VisibleObstacle, rectangle_polygon, vehicle_step,
    )

try:
    from local_frame import LocalSide, get_local_side
    from planner_tuning import PlannerTuning
except ImportError:
    from simulator.local_frame import LocalSide, get_local_side
    from simulator.planner_tuning import PlannerTuning


BOUNDARY = rectangle_polygon((0.0, 0.0, 300.0, 300.0))


class GeometricPlannerTests(unittest.TestCase):
    def test_execution_horizon_is_a_distance(self):
        controller = AutonomousController(GeometricPlanner())
        state = VehicleState(125.0, 280.0, 0.0, 24.0, 0.0)

        self.assertEqual(controller.execution_horizon_cm(state), 6.0)
        self.assertAlmostEqual(controller.execution_interval_s(state, 24.0), 0.25)

    def test_local_left_right_rotates_with_each_straight(self):
        cases = (
            ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0)),   # top: avance derecha
            ((0.0, 1.0), (-1.0, 0.0), (1.0, 0.0)),   # right: avance abajo
            ((-1.0, 0.0), (0.0, -1.0), (0.0, 1.0)), # bottom: avance izquierda
            ((0.0, -1.0), (1.0, 0.0), (-1.0, 0.0)), # left: avance arriba
        )
        for forward, right_point, left_point in cases:
            with self.subTest(forward=forward):
                self.assertIs(
                    get_local_side(right_point, (0.0, 0.0), forward),
                    LocalSide.RIGHT,
                )
                self.assertIs(
                    get_local_side(left_point, (0.0, 0.0), forward),
                    LocalSide.LEFT,
                )

    def test_geometry_comes_from_physical_measurements(self):
        geometry = VehicleGeometry()
        self.assertEqual((geometry.length_cm, geometry.width_cm, geometry.wheelbase_cm), (21.15, 17.2, 15.0))
        self.assertEqual((geometry.minimum_right_radius_cm, geometry.minimum_left_radius_cm), (32.2, 43.0))

    def test_predictions_cover_perception_range(self):
        planner = GeometricPlanner()
        data = PlannerInput(VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY)

        self.assertEqual(planner._planning_distance(data, None), 50.0)
        self.assertTrue(all(
            math.isclose(
                sum(primitive.distance_cm for primitive in candidate.primitives),
                50.0,
            )
            for candidate in planner._forward_candidates(data)
        ))

    def test_incremental_beam_does_not_resimulate_complete_children(self):
        planner = GeometricPlanner(tuning=PlannerTuning(
            planning_budget_mode="candidate_count",
            diagnostic_level="summary",
        ))
        obstacle = VisibleObstacle("1", 75.0, 100.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        # 5 + 20 + (4 * 5) segmentos del beam y un segmento del diagnóstico
        # frontal; ningún candidato final vuelve a simularse completo.
        self.assertEqual(result.diagnostics.simulation_calls, 1)
        self.assertEqual(result.diagnostics.segment_simulations, 46)
        self.assertEqual(result.diagnostics.clearance_evaluations, 687)
        self.assertTrue(all(not candidate.trajectory_points
                            for candidate in result.candidates))

    def test_incremental_paths_match_full_resimulation_reference(self):
        planner = GeometricPlanner(tuning=PlannerTuning(
            planning_budget_mode="candidate_count",
            diagnostic_level="full",
        ))
        obstacle = VisibleObstacle("1", 75.0, 100.0, 5.0, 5.0, "red")
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        )
        result = planner.plan(data)
        cache = planner._geometry_cache(data)
        for incremental in result.candidates:
            if not incremental.candidate_id.startswith("BEAM:"):
                continue
            reference = CandidateTrajectory(
                incremental.candidate_id, incremental.profile,
                incremental.primitives,
                target_obstacle_id=incremental.target_obstacle_id,
                desired_pass_side=incremental.desired_pass_side,
            )
            planner.simulate(data.vehicle_state, reference, cache)
            planner.validate(reference, data, cache=cache)
            self.assertEqual(incremental.safe, reference.safe)
            self.assertAlmostEqual(incremental.score, reference.score, places=6)

    def test_fast_path_preserves_straight_decision_and_is_disabled_by_obstacle(self):
        planner = GeometricPlanner(tuning=PlannerTuning(
            planning_budget_mode="candidate_count",
        ))
        clear = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY,
            desired_heading_rad=0.0,
        ))
        self.assertTrue(clear.diagnostics.fast_path)
        self.assertEqual(clear.command.steering_angle_deg, 0.0)

        obstacle = VisibleObstacle("ahead", 100.0, 50.0, 5.0, 5.0, "red")
        blocked = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        self.assertFalse(blocked.diagnostics.fast_path)

    def test_all_visible_obstacles_are_cached_and_nearest_is_priority(self):
        planner = GeometricPlanner()
        near = VisibleObstacle("near", 75.0, 50.0, 5.0, 5.0, "red")
        far = VisibleObstacle("far", 110.0, 50.0, 5.0, 5.0, "green")
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (far, near), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        )
        self.assertEqual(planner._nearest(data).object_id, "near")
        self.assertEqual(len(planner._geometry_cache(data).obstacle_polygons), 2)

    def test_tracked_obstacle_remains_available_after_leaving_fov(self):
        planner = GeometricPlanner()
        remembered = VisibleObstacle("remembered", 80.0, 50.0, 5.0, 5.0, "red")
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0, 0.0, (), (remembered,),
        )
        self.assertEqual(planner._nearest(data).object_id, "remembered")
        self.assertEqual(len(planner._geometry_cache(data).obstacle_polygons), 1)

    def test_controller_releases_passed_target_and_can_select_next(self):
        controller = AutonomousController(GeometricPlanner())
        target = VisibleObstacle("1", 75.0, 50.0, 5.0, 5.0, "red")
        controller.active_target_id = target.object_id
        controller._tracked_obstacles[target.object_id] = target
        data = PlannerInput(
            VehicleState(100.0, 65.0, 0.0), (), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0, 0.0,
            ((50.0, 50.0), (150.0, 50.0)),
        )
        controller._update_active_target(data)
        self.assertIsNone(controller.active_target_id)

    def test_reverse_stops_after_smallest_successful_recovery(self):
        planner = GeometricPlanner(tuning=PlannerTuning(
            planning_budget_mode="candidate_count",
            diagnostic_level="summary",
        ))
        obstacle = VisibleObstacle("near", 75.0, 70.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        self.assertEqual(result.diagnostics.reverse_distance_cm, 5.0)
        self.assertTrue(result.best_candidate.candidate_id.startswith("RECOVERY_5CM:"))
        self.assertFalse(any(
            candidate.candidate_id.startswith("REVERSE_10CM")
            or candidate.candidate_id.startswith("RECOVERY_10CM")
            for candidate in result.candidates
        ))

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
        planner = GeometricPlanner(
            tuning=PlannerTuning(planning_budget_mode="candidate_count")
        )
        data = PlannerInput(VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY)
        result = planner.plan(data)
        self.assertEqual(result.state, PlannerState.FOLLOW)
        self.assertEqual(
            [primitive.kind for primitive in result.best_candidate.primitives],
            [PrimitiveType.STRAIGHT] * 3,
        )
        self.assertTrue(result.best_candidate.candidate_id.startswith("BEAM:"))
        self.assertGreater(result.command.target_speed_cm_s, 0.0)
        self.assertEqual(result.command.steering_angle_deg, 0.0)

    def test_route_heading_is_the_primary_local_tangent(self):
        planner = GeometricPlanner()
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), drivable_boundary=BOUNDARY,
            desired_heading_rad=math.pi / 2,
        )
        self.assertAlmostEqual(planner._track_tangent(data), math.pi / 2)

    def test_target_tangent_comes_from_the_obstacle_route_segment(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("top", 150.0, 50.0, 5.0, 5.0, "green")
        data = PlannerInput(
            VehicleState(50.0, 150.0, -math.pi / 2), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, -math.pi / 2, 0.0,
            ((50.0, 150.0), (50.0, 50.0), (250.0, 50.0)),
        )
        self.assertAlmostEqual(planner._target_tangent(data, obstacle), 0.0)

    def test_blocked_projection_generates_complete_beam_trajectories(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("1", 75.0, 70.0, 5.0, 5.0, "red")
        data = PlannerInput(VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
                            TrackDirection.CLOCKWISE, 0.0)
        candidates = planner._forward_candidates(data)
        self.assertLessEqual(len(candidates), 4)
        self.assertTrue(candidates)
        self.assertTrue(all(len(candidate.primitives) == 3
                            for candidate in candidates))
        self.assertTrue(all(math.isclose(
            sum(primitive.distance_cm for primitive in candidate.primitives),
            50.0,
        ) for candidate in candidates))
        self.assertTrue(all(candidate.trajectory_points
                            for candidate in candidates))
        self.assertTrue(any(
            any(primitive.kind is PrimitiveType.ARC_LEFT
                for primitive in candidate.primitives)
            for candidate in candidates
        ))
        self.assertTrue(any(
            any(primitive.kind is PrimitiveType.ARC_RIGHT
                for primitive in candidate.primitives)
            for candidate in candidates
        ))

    def test_visible_colored_obstacle_is_targeted_before_legacy_distance_trigger(self):
        """La primera detección visible genera perfiles, incluso a 100 cm."""
        planner = GeometricPlanner(
            tuning=PlannerTuning(planning_budget_mode="candidate_count")
        )
        obstacle = VisibleObstacle("far", 150.0, 50.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        self.assertTrue(result.candidates)
        self.assertTrue(all(len(candidate.primitives) == 3
                            for candidate in result.candidates))
        self.assertTrue(all(math.isclose(
            sum(primitive.distance_cm for primitive in candidate.primitives),
            50.0,
        ) for candidate in result.candidates))
        self.assertFalse(any(
            candidate.primitives[0].kind is PrimitiveType.REVERSE
            for candidate in result.candidates
        ))

    def test_reverse_is_evaluated_only_after_all_forward_candidates_fail(self):
        planner = GeometricPlanner(
            tuning=PlannerTuning(planning_budget_mode="candidate_count")
        )
        # No hay forward válido; el recovery mínimo que abre un forward gana.
        obstacle = VisibleObstacle("near", 80.0, 45.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        forward_valid = any(
            candidate.safe and candidate.primitives[0].kind is not PrimitiveType.REVERSE
            for candidate in result.candidates
        )
        reverse_evaluated = any(
            candidate.primitives[0].kind is PrimitiveType.REVERSE
            for candidate in result.candidates
        )
        self.assertFalse(forward_valid)
        self.assertTrue(reverse_evaluated)
        self.assertTrue(result.best_candidate.candidate_id.startswith("RECOVERY_"))
        self.assertIn(result.diagnostics.reverse_distance_cm, (2.0, 5.0, 10.0))

    def test_passed_object_remains_collision_geometry_not_a_new_target(self):
        planner = GeometricPlanner()
        passed = VisibleObstacle(
            "passed", 75.0, 50.0, 5.0, 5.0, "red", 0.0, True,
        )
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (passed,), (), BOUNDARY,
        )
        self.assertIsNone(planner._nearest(data))
        result = planner.plan(data)
        self.assertFalse(result.candidates[0].safe)

    def test_active_target_is_kept_when_heading_changes(self):
        planner = GeometricPlanner()
        active = VisibleObstacle("active", 40.0, 50.0, 5.0, 5.0, "green")
        data = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (active,), (), BOUNDARY,
            active_target_id="active",
        )
        self.assertEqual(planner._nearest(data), active)

    def test_disable_margins_never_allows_a_candidate_outside_track(self):
        planner = GeometricPlanner(tuning=PlannerTuning(
            planning_budget_mode="candidate_count",
            disable_hard_safety_margins=True,
            allow_physical_collisions=True,
        ))
        result = planner.plan(PlannerInput(
            VehicleState(290.0, 150.0, 0.0), drivable_boundary=BOUNDARY,
        ))
        straight = result.candidates[0]
        self.assertFalse(straight.safe)
        self.assertEqual(straight.diagnostic_rejection_reason, "OUT_OF_TRACK")
        self.assertEqual(result.state, PlannerState.NO_SAFE_TRAJECTORY)
        self.assertEqual(
            result.diagnostics.no_safe_reason,
            "ALL_FORWARD_AND_REVERSE_INVALID",
        )
        self.assertEqual(result.diagnostics.no_safe_detail, "OUT_OF_TRACK")
        self.assertEqual(result.command.target_speed_cm_s, 0.0)

    def test_pending_pass_side_is_diagnostic_not_hard_rejection(self):
        """No se rechaza una trayectoria solo por una estimación lateral."""
        planner = GeometricPlanner()
        initial = VehicleState(50.0, 50.0, 0.0)

        def candidate_at(y_cm: float) -> CandidateTrajectory:
            candidate = CandidateTrajectory(
                "test", None,
                (MotionPrimitive(PrimitiveType.STRAIGHT, 10.0, 0.0, 18.0),),
            )
            planner.simulate(initial, candidate)
            # La trayectoria simulada recta se sustituye por la pose de
            # aproximación que queremos clasificar geométricamente.
            state = VehicleState(66.0, y_cm, 0.0)
            candidate.trajectory_points = candidate.trajectory_points[:1] + [
                TrajectoryPoint(
                    state, 0, 10.0, planner.geometry.footprint(state),
                )
            ]
            return candidate

        red = VisibleObstacle("red", 120.0, 50.0, 5.0, 5.0, "red")
        green = VisibleObstacle("green", 120.0, 50.0, 5.0, 5.0, "green")
        red_data = PlannerInput(initial, (red,), (), BOUNDARY,
                                TrackDirection.CLOCKWISE, 0.0)
        green_data = PlannerInput(initial, (green,), (), BOUNDARY,
                                  TrackDirection.CLOCKWISE, 0.0)

        red_candidate = candidate_at(40.0)
        green_candidate = candidate_at(60.0)
        planner.validate(red_candidate, red_data)
        planner.validate(green_candidate, green_data)
        self.assertTrue(red_candidate.safe)
        self.assertTrue(green_candidate.safe)

    def test_red_and_green_pass_targets_rotate_with_all_four_straights(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("target", 100.0, 100.0, 5.0, 5.0, "red")
        directions = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
        for forward in directions:
            with self.subTest(forward=forward):
                heading = math.atan2(forward[1], forward[0])
                right = (-forward[1], forward[0])
                state = VehicleState(
                    100.0 - forward[0] * 50.0,
                    100.0 - forward[1] * 50.0,
                    heading,
                )
                route = (
                    (state.x_cm, state.y_cm),
                    (100.0, 100.0),
                    (100.0 + forward[0] * 50.0, 100.0 + forward[1] * 50.0),
                )
                data = PlannerInput(state, (obstacle,), (), BOUNDARY,
                                    TrackDirection.CLOCKWISE, heading, 0.0, route)
                red = planner._pass_target(data, obstacle)
                self.assertEqual(red.side, LocalSide.RIGHT)
                self.assertGreater(red.target_lateral_offset_cm, 0.0)
                self.assertEqual(planner._steering_towards_pass_target(data, obstacle, red), 1)
                green = planner._pass_target(data, VisibleObstacle(
                    "target", 100.0, 100.0, 5.0, 5.0, "green",
                ))
                self.assertEqual(green.side, LocalSide.LEFT)
                self.assertLess(green.target_lateral_offset_cm, 0.0)
                self.assertEqual(planner._steering_towards_pass_target(data, obstacle, green), -1)

    def test_red_right_boundary_accepts_extra_clearance(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("red", 100.0, 100.0, 5.0, 5.0, "red")
        state = VehicleState(50.0, 122.0, 0.0)
        data = PlannerInput(state, (obstacle,), (), BOUNDARY,
                            TrackDirection.CLOCKWISE, 0.0, 0.0,
                            ((50.0, 100.0), (150.0, 100.0)))
        target = planner._pass_target(data, obstacle)
        self.assertEqual(target.side, LocalSide.RIGHT)
        self.assertEqual(planner._steering_towards_pass_target(data, obstacle, target), 0)

    def test_green_left_boundary_accepts_extra_clearance(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("green", 100.0, 100.0, 5.0, 5.0, "green")
        state = VehicleState(50.0, 78.0, 0.0)
        data = PlannerInput(state, (obstacle,), (), BOUNDARY,
                            TrackDirection.CLOCKWISE, 0.0, 0.0,
                            ((50.0, 100.0), (150.0, 100.0)))
        target = planner._pass_target(data, obstacle)
        self.assertEqual(target.side, LocalSide.LEFT)
        self.assertEqual(planner._steering_towards_pass_target(data, obstacle, target), 0)

    def test_straight_is_not_rejected_by_pass_side_estimation(self):
        planner = GeometricPlanner()
        initial = VehicleState(50.0, 50.0, 0.0)

        def straight_to(obstacle_x: float) -> CandidateTrajectory:
            candidate = planner._straight(30.0)
            planner.simulate(initial, candidate)
            return candidate

        near = VisibleObstacle("near", 100.0, 50.0, 5.0, 5.0, "red")
        far = VisibleObstacle("far", 170.0, 50.0, 5.0, 5.0, "red")
        near_data = PlannerInput(initial, (near,), (), BOUNDARY,
                                 TrackDirection.CLOCKWISE, 0.0,
                                 route_centerline=((50.0, 50.0), (200.0, 50.0)))
        far_data = PlannerInput(initial, (far,), (), BOUNDARY,
                                TrackDirection.CLOCKWISE, 0.0,
                                route_centerline=((50.0, 50.0), (200.0, 50.0)))
        near_candidate = straight_to(100.0)
        planner.validate(near_candidate, near_data)
        self.assertTrue(near_candidate.safe)
        far_candidate = straight_to(170.0)
        planner.validate(far_candidate, far_data)
        self.assertTrue(far_candidate.safe)

    def test_complete_pass_uses_full_footprint_side_for_red_and_green(self):
        planner = GeometricPlanner()
        initial = VehicleState(50.0, 50.0, 0.0)

        def completed_at(y_cm: float) -> CandidateTrajectory:
            candidate = CandidateTrajectory("complete", None, (
                MotionPrimitive(PrimitiveType.STRAIGHT, 10.0, 0.0, 18.0),
            ))
            state = VehicleState(120.0, y_cm, 0.0)
            candidate.trajectory_points = [TrajectoryPoint(
                state, 0, 70.0, planner.geometry.footprint(state),
            )]
            return candidate

        route = ((50.0, 50.0), (150.0, 50.0))
        red = VisibleObstacle("red", 75.0, 50.0, 5.0, 5.0, "red")
        green = VisibleObstacle("green", 75.0, 50.0, 5.0, 5.0, "green")
        red_data = PlannerInput(initial, (red,), (), BOUNDARY,
                                TrackDirection.CLOCKWISE, 0.0, 0.0, route)
        green_data = PlannerInput(initial, (green,), (), BOUNDARY,
                                  TrackDirection.CLOCKWISE, 0.0, 0.0, route)
        self.assertTrue(planner._correct_side(completed_at(80.0), red_data))
        self.assertFalse(planner._correct_side(completed_at(20.0), red_data))
        self.assertTrue(planner._correct_side(completed_at(20.0), green_data))
        self.assertFalse(planner._correct_side(completed_at(80.0), green_data))

    def test_geometrically_correct_straight_is_not_cancelled_by_heuristics(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("red", 100.0, 50.0, 5.0, 5.0, "red")
        data = PlannerInput(
            VehicleState(50.0, 80.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0, 0.0,
            ((50.0, 50.0), (160.0, 50.0)),
        )
        candidate = planner._straight(80.0)
        planner.simulate(data.vehicle_state, candidate)
        planner.validate(candidate, data)
        self.assertTrue(candidate.safe)
        self.assertTrue(candidate.correct_pass_side)

    def test_target_tangent_uses_the_closest_segment_at_a_corner(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("corner", 55.0, 70.0, 5.0, 5.0, "red")
        route = ((50.0, 150.0), (50.0, 50.0), (250.0, 50.0))
        data = PlannerInput(VehicleState(50.0, 150.0, -math.pi / 2),
                            (obstacle,), (), BOUNDARY, TrackDirection.CLOCKWISE,
                            -math.pi / 2, 0.0, route)
        self.assertAlmostEqual(planner._target_tangent(data, obstacle), -math.pi / 2)

    def test_colored_avoidance_uses_complete_ackermann_sequences(self):
        planner = GeometricPlanner(
            tuning=PlannerTuning(planning_budget_mode="candidate_count")
        )
        obstacle = VisibleObstacle("red", 75.0, 70.0, 5.0, 5.0, "red")
        result = planner.plan(PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        ))
        forward = [candidate for candidate in result.candidates
                   if candidate.candidate_id.startswith("BEAM:")]
        self.assertTrue(forward)
        self.assertTrue(all(len(candidate.primitives) == 3
                            for candidate in forward))
        self.assertTrue(any(
            any(primitive.kind is PrimitiveType.ARC_RIGHT
                for primitive in candidate.primitives)
            for candidate in forward
        ))

    def test_swept_footprint_detects_obstacle_not_only_center(self):
        planner = GeometricPlanner()
        obstacle = VisibleObstacle("edge", 70.0, 58.0, 5.0, 5.0, "unknown")
        data = PlannerInput(VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY)
        straight = planner._straight(50.0)
        planner.simulate(data.vehicle_state, straight)
        planner.validate(straight, data)
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

    def test_controller_commits_complete_trajectory_while_remaining_path_is_safe(self):
        planner = GeometricPlanner(
            tuning=PlannerTuning(planning_budget_mode="candidate_count")
        )
        controller = AutonomousController(planner)
        obstacle = VisibleObstacle("red", 120.0, 50.0, 5.0, 5.0, "red")
        initial = PlannerInput(
            VehicleState(50.0, 50.0, 0.0), (obstacle,), (), BOUNDARY,
            TrackDirection.CLOCKWISE, 0.0,
        )
        first = controller.plan(initial)
        self.assertIsNotNone(first.best_candidate)
        forward = [
            candidate for candidate in first.candidates
            if candidate.primitives[0].kind is not PrimitiveType.REVERSE
        ]
        self.assertTrue(forward)
        self.assertTrue(all(math.isclose(
            sum(primitive.distance_cm for primitive in candidate.primitives),
            50.0,
        ) for candidate in forward))

        advanced = vehicle_step(initial.vehicle_state, first.command, 0.2, planner.geometry)
        second = controller.plan(PlannerInput(
            advanced, (obstacle,), (), BOUNDARY, TrackDirection.CLOCKWISE, 0.0,
        ))
        self.assertTrue(second.best_candidate)
        self.assertEqual(second.diagnostics.commitment_mode, "flexible")
        self.assertEqual(
            second.diagnostics.committed_candidate_id,
            second.best_candidate.candidate_id,
        )
        if second.best_candidate.candidate_id == first.best_candidate.candidate_id:
            self.assertLess(
                second.best_candidate.primitives[0].distance_cm,
                first.best_candidate.primitives[0].distance_cm,
            )

    def test_flexible_commitment_requires_switch_margin(self):
        planner = GeometricPlanner(
            tuning=PlannerTuning(
                planning_budget_mode="candidate_count",
                switch_margin=8.0,
            )
        )
        controller = AutonomousController(planner)
        primitive = MotionPrimitive(PrimitiveType.STRAIGHT, 10.0, 0.0, 18.0)
        current = CandidateTrajectory("current", None, (primitive,), safe=True, score=84.0)
        slightly_better = CandidateTrajectory("new", None, (primitive,), safe=True, score=85.0)
        clearly_better = CandidateTrajectory("new", None, (primitive,), safe=True, score=96.0)

        self.assertFalse(controller._should_switch(current, slightly_better))
        self.assertTrue(controller._should_switch(current, clearly_better))
        self.assertFalse(controller._should_switch(current, None))

    def test_candidate_count_mode_repeats_identically(self):
        """La misma seed debe producir la misma ejecución completa."""
        runs = [
            run_scenario(
                seed=20260815,
                scenario_index=2,
                sensor=SensorModel(),
                duration_s=8.0,
                planning_budget_mode="candidate_count",
            )[0]
            for _ in range(3)
        ]

        def deterministic_data(summary):
            return {
                key: summary[key]
                for key in (
                    "seed", "scenario", "objects", "single_obstacle_straight", "collision", "straight_progress",
                    "route_progress_valid", "correct_pass_side", "termination_reason",
                    "initial_pose", "track_direction", "planning_budget_mode",
                    "cycles",
                )
            }

        expected = deterministic_data(runs[0])
        for run in runs[1:]:
            self.assertEqual(deterministic_data(run), expected)

    def test_runner_modes_and_three_lap_target(self):
        clear, _ = run_scenario(1, 0, SensorModel(), 0.1, mode="1")
        obstacles, _ = run_scenario(1, 0, SensorModel(), 0.1, mode="2")
        self.assertEqual(clear["objects"], [])
        self.assertGreater(len(obstacles["objects"]), 0)
        self.assertEqual(clear["target_straights"], 12)
        self.assertEqual(clear["laps_completed"], 0)
        with self.assertRaises(ValueError):
            run_scenario(1, 0, SensorModel(), 0.1, mode="3")

    def test_runner_seed_derivation_and_margin_override(self):
        summary, _ = run_scenario(
            seed=20260816, scenario_index=1, sensor=SensorModel(),
            duration_s=0.1, base_seed=20260815,
            disable_safety_margins=True,
        )
        self.assertEqual(summary["scenario_index"], 1)
        self.assertEqual(summary["base_seed"], 20260815)
        self.assertEqual(summary["effective_seed"], 20260816)
        self.assertTrue(summary["safety_margins_disabled"])


def planner_command(speed: float, steering: float):
    from simulator.geometric_planner import ControlCommand
    return ControlCommand(speed, steering)


if __name__ == "__main__":
    unittest.main()
