"""Command-to-public-AirStack-task routing tests."""
import unittest

from rrm.airstack_command import (
    CommandClarificationRequired, CommandEnvironment, EXPLORATION_ACTION, LAND_ACTION,
    NAVIGATE_ACTION, ParameterSource, TAKEOFF_ACTION, ground_command, plan_command,
    takeoff_recovery_action,
)
from rrm.airstack_drone import DroneTaskKind


ALL = {TAKEOFF_ACTION, LAND_ACTION, NAVIGATE_ACTION, EXPLORATION_ACTION}


class AirStackCommandTests(unittest.TestCase):
    def test_measured_climb_and_descend_are_preserved_as_vertical_intents(self):
        for verb, direction, delta in (("climb", "up", .5), ("descend", "down", -.5)):
            for connector in ("", "by ", "for "):
                with self.subTest(verb=verb, connector=connector):
                    grounded = ground_command(
                        f"takeoff to 1m, fly forward for 1m, then {verb} {connector}0.5m, then land",
                        task_id="t", robot_name="robot_1", action_servers=ALL,
                        airborne=False, environment=CommandEnvironment(
                            current_position=(2., 3., .02), yaw_rad=0.),
                    )
                    self.assertEqual([a.kind.value for a in grounded.actions],
                                     ["TAKEOFF", "NAVIGATE", "NAVIGATE", "LAND"])
                    self.assertEqual([(i.direction, i.distance_m)
                                      for i in grounded.relative_navigation_bindings],
                                     [("forward", 1.), (direction, .5)])
                    goal = grounded.actions[2].waypoints[0]
                    self.assertEqual((goal.x, goal.y, goal.z), (3., 3., 1.+delta))
                    self.assertEqual(grounded.actions[2].goal_tolerance_m, .05)

    def test_unmeasured_vertical_verbs_never_silently_disappear(self):
        for clause in ("climb", "climb a little", "climb for 4 seconds",
                       "descend", "descend for a bit", "climb to 1.5m",
                       "climb -0.5m", "descend 0m"):
            with self.subTest(clause=clause), self.assertRaises(CommandClarificationRequired):
                ground_command(f"takeoff to 1m, fly forward 1m, then {clause}, then land",
                               task_id="t", robot_name="robot_1", action_servers=ALL,
                               airborne=False, environment=CommandEnvironment(
                                   current_position=(0., 0., .02), yaw_rad=0.))

    def test_vertical_synonyms_retain_action_order_guard(self):
        with self.assertRaisesRegex(CommandClarificationRequired, "interleaved"):
            ground_command("fly forward 1m, land, then climb .5m", task_id="t",
                           robot_name="robot_1", action_servers=ALL, airborne=True,
                           environment=CommandEnvironment(current_position=(0., 0., 1.), yaw_rad=0.))

    def test_relative_distance_is_not_a_second_exploration_duration(self):
        for distance in ("1m", "1 m", "12 meters", "1 metre", "2.5 metres"):
            with self.subTest(distance=distance):
                grounded = ground_command(
                    f"takeoff to 1m, fly forward for {distance}, explore for 4 seconds, then land",
                    task_id="t1", robot_name="robot_1", action_servers=ALL,
                    airborne=False, environment=CommandEnvironment(
                        current_position=(0., 0., .01), yaw_rad=0.),
                )
                self.assertEqual([a.kind for a in grounded.actions], [
                    DroneTaskKind.TAKEOFF, DroneTaskKind.NAVIGATE,
                    DroneTaskKind.EXPLORE, DroneTaskKind.LAND])
                self.assertEqual(grounded.actions[2].time_limit_s, 4.)
                duration = next(p for p in grounded.parameter_grounding
                                if p.parameter == "time_limit_s")
                self.assertEqual(duration.requested_value, 4.)

    def test_duration_units_and_actual_multiple_timers_remain_distinct(self):
        for phrase, seconds in (("explore for 12 seconds", 12.),
                                ("explore for 2 minutes", 120.),
                                ("explore for 12", 12.)):
            self.assertEqual(plan_command(
                phrase, task_id="t1", robot_name="robot_1", action_servers=ALL,
                airborne=True)[0].time_limit_s, seconds)
        with self.assertRaisesRegex(CommandClarificationRequired, "single exploration duration"):
            plan_command("explore for 4 seconds during 2 minutes", task_id="t1",
                         robot_name="robot_1", action_servers=ALL, airborne=True)

    def test_explicit_four_second_exploration_is_not_extended(self):
        grounded = ground_command(
            "takeoff to 1m, explore for 4 seconds, then land", task_id="t1",
            robot_name="robot_1", action_servers=ALL, airborne=False,
        )
        self.assertEqual([a.kind for a in grounded.actions], [
            DroneTaskKind.TAKEOFF, DroneTaskKind.EXPLORE, DroneTaskKind.LAND])
        self.assertEqual(grounded.actions[1].time_limit_s, 4.0)
        duration = next(p for p in grounded.parameter_grounding
                        if p.parameter == "time_limit_s")
        self.assertEqual(duration.source, ParameterSource.OPERATOR_EXPLICIT)
        self.assertEqual(duration.requested_value, 4.0)
        for value in (0, 0.5, 901):
            with self.assertRaises(CommandClarificationRequired):
                ground_command(f"explore for {value} seconds", task_id="bad",
                               robot_name="robot_1", action_servers=ALL, airborne=True)

    def test_compound_relative_command_preserves_forward_clause(self):
        plan = plan_command(
            "takeoff to 1m, fly forward for 1m, then land",
            task_id="t1", robot_name="robot_1", action_servers=ALL,
            airborne=False, current_position=(2.0, 3.0, 0.02), yaw_rad=0.0,
        )
        self.assertEqual([item.kind for item in plan], [
            DroneTaskKind.TAKEOFF, DroneTaskKind.NAVIGATE, DroneTaskKind.LAND,
        ])
        self.assertEqual(plan[0].target_altitude_m, 1.0)
        point = plan[1].waypoints[0]
        self.assertEqual((point.x, point.y, point.z), (3.0, 3.0, 1.0))

    def test_unparsed_relative_clause_does_not_silently_disappear(self):
        for phrase in ("fly forward", "fly forward for a bit", "fly forward for 4 seconds",
                       "fly forward 1m, then move left"):
            with self.subTest(phrase=phrase):
                with self.assertRaises(CommandClarificationRequired):
                    plan_command(
                        f"takeoff to 1m, {phrase}, then land", task_id="t1",
                        robot_name="robot_1", action_servers=ALL, airborne=False,
                        current_position=(0.0, 0.0, 0.02), yaw_rad=0.0,
                    )

    def test_two_relative_legs_have_reviewed_intents_and_short_tolerances(self):
        grounded = ground_command(
            "takeoff to 1m, fly forward for 1m, then move left for 1m, "
            "explore for 6 seconds, then land", task_id="t1", robot_name="robot_1",
            action_servers=ALL, airborne=False,
            environment=CommandEnvironment(current_position=(2., 3., .02), yaw_rad=0.),
        )
        self.assertEqual([a.kind.value for a in grounded.actions],
                         ["TAKEOFF", "NAVIGATE", "NAVIGATE", "EXPLORE", "LAND"])
        self.assertEqual([(b.action_id, b.direction, b.distance_m)
                          for b in grounded.relative_navigation_bindings],
                         [("navigate-1", "forward", 1.), ("navigate-2", "left", 1.)])
        self.assertEqual([a.goal_tolerance_m for a in grounded.actions[1:3]], [.1, .1])
        self.assertEqual(grounded.actions[3].time_limit_s, 6.)

    def test_relative_interleaving_is_rejected_not_reordered(self):
        for phrase in ("fly forward 1m, explore for 4 seconds, then move left 1m",
                       "fly forward 1m, land, then move left 1m",
                       "explore for 4 seconds, then fly forward 1m"):
            with self.subTest(phrase=phrase), self.assertRaises(CommandClarificationRequired):
                plan_command(phrase, task_id="t", robot_name="robot_1", action_servers=ALL,
                             airborne=True, current_position=(0., 0., 1.), yaw_rad=0.)

    def test_measured_go_back_is_not_an_additional_return_home(self):
        plan = plan_command("go back 1m, then land", task_id="t", robot_name="robot_1",
                            action_servers=ALL, airborne=True,
                            current_position=(0., 0., 1.), yaw_rad=0.)
        self.assertEqual([a.kind.value for a in plan], ["NAVIGATE", "LAND"])

    def test_relative_clause_does_not_remove_explicit_return_home(self):
        plan = plan_command("fly forward 1m and return home, then land", task_id="t",
                            robot_name="robot_1", action_servers=ALL, airborne=True,
                            current_position=(0., 0., 1.), yaw_rad=0.)
        self.assertEqual([a.kind.value for a in plan], ["NAVIGATE", "NAVIGATE", "LAND"])

    def test_grounded_exploration_uses_air_stacks_global_task_cascade(self):
        plan = plan_command("Take off and explore for 30 seconds, then land",
                            task_id="t1", robot_name="robot_1",
                            action_servers=ALL, airborne=False)
        self.assertEqual([item.kind for item in plan], [
            DroneTaskKind.TAKEOFF, DroneTaskKind.EXPLORE, DroneTaskKind.LAND,
        ])
        self.assertEqual(plan[1].action_name, "/robot_1/tasks/exploration")
        self.assertEqual(plan[1].time_limit_s, 30.0)

    def test_map_coordinate_navigation_auto_inserts_takeoff(self):
        plan = plan_command("fly to x=2 y=-1 z=1.5", task_id="t1",
                            robot_name="robot_1", action_servers=ALL, airborne=False)
        self.assertEqual([item.kind for item in plan], [
            DroneTaskKind.TAKEOFF, DroneTaskKind.NAVIGATE,
        ])
        self.assertEqual(plan[-1].waypoints[0].y, -1.0)
        recovery = takeoff_recovery_action(plan)
        self.assertEqual(recovery.kind, DroneTaskKind.LAND)
        self.assertEqual(recovery.action_id, "takeoff-0-recovery-land")

    def test_grounded_motion_requires_a_recovery_land_executor(self):
        with self.assertRaisesRegex(ValueError, "safe recovery landing"):
            plan_command(
                "fly to x=2 y=-1 z=1.5", task_id="t1", robot_name="robot_1",
                action_servers={TAKEOFF_ACTION, NAVIGATE_ACTION}, airborne=False,
            )

    def test_multiple_map_waypoints_are_one_public_navigation_task(self):
        plan = plan_command("fly through (1, 2, 1.5), (4, -2, 2)", task_id="t1",
                            robot_name="robot_1", action_servers=ALL, airborne=True)
        self.assertEqual([item.kind for item in plan], [DroneTaskKind.NAVIGATE])
        self.assertEqual([(point.x, point.y, point.z) for point in plan[0].waypoints],
                         [(1.0, 2.0, 1.5), (4.0, -2.0, 2.0)])
        self.assertIsNone(takeoff_recovery_action(plan))

    def test_relative_command_uses_fresh_pose_and_heading(self):
        plan = plan_command("move forward 2 meters", task_id="t1", robot_name="robot_1",
                            action_servers=ALL, airborne=True,
                            current_position=(1.0, 2.0, 1.5), yaw_rad=0.0)
        point = plan[0].waypoints[0]
        self.assertAlmostEqual(point.x, 3.0)
        self.assertAlmostEqual(point.y, 2.0)
        self.assertEqual(point.z, 1.5)

    def test_relative_command_fails_without_live_heading(self):
        with self.assertRaisesRegex(ValueError, "position and heading"):
            plan_command("move left 2m", task_id="t1", robot_name="robot_1",
                         action_servers=ALL, airborne=True)

    def test_grounded_relative_command_flies_at_inserted_takeoff_altitude(self):
        plan = plan_command("move forward 2m", task_id="t1", robot_name="robot_1",
                            action_servers=ALL, airborne=False,
                            current_position=(0.0, 0.0, 0.0), yaw_rad=0.0)
        self.assertEqual([item.kind for item in plan],
                         [DroneTaskKind.TAKEOFF, DroneTaskKind.NAVIGATE])
        self.assertEqual(plan[1].waypoints[0].z, 1.5)

    def test_grounded_land_is_already_satisfied(self):
        with self.assertRaisesRegex(ValueError, "already satisfied"):
            plan_command("land", task_id="t1", robot_name="robot_1",
                         action_servers=ALL, airborne=False)

    def test_only_real_servers_are_routable(self):
        with self.assertRaisesRegex(ValueError, "no global exploration"):
            plan_command("explore", task_id="t1", robot_name="robot_1",
                         action_servers={TAKEOFF_ACTION}, airborne=False)

    def test_unsupported_free_text_never_becomes_motion(self):
        with self.assertRaisesRegex(ValueError, "Unsupported movement command"):
            plan_command("do something interesting", task_id="t1", robot_name="robot_1",
                         action_servers=ALL, airborne=False)

    def test_vague_live_goal_is_grounded_bounded_and_returns_to_start(self):
        grounded = ground_command(
            "Fly and explore the world for a couple of seconds, then come back and land",
            task_id="t1", robot_name="robot_1", action_servers=ALL, airborne=False,
            environment=CommandEnvironment(
                active_scene="office", current_position=(0.2, -0.4, 0.02), yaw_rad=0.0,
                map_fresh=True, map_point_count=852,
                map_bounds_xy=(-5.0, 5.0, -4.0, 4.0),
            ),
        )
        self.assertEqual([item.kind for item in grounded.actions], [
            DroneTaskKind.TAKEOFF, DroneTaskKind.EXPLORE,
            DroneTaskKind.NAVIGATE, DroneTaskKind.LAND,
        ])
        explore = grounded.actions[1]
        self.assertEqual(explore.time_limit_s, 5.0)
        self.assertEqual(len(explore.search_bounds), 4)
        self.assertEqual(
            [(point.x, point.y, point.z) for point in grounded.actions[2].waypoints],
            [(0.2, -0.4, 1.5)],
        )
        duration = next(item for item in grounded.parameter_grounding
                        if item.action_id == explore.action_id
                        and item.parameter == "time_limit_s")
        self.assertEqual(duration.requested_value, 2.0)
        self.assertEqual(duration.source, ParameterSource.VEHICLE_ENVELOPE)
        bounds = next(item for item in grounded.parameter_grounding
                      if item.action_id == explore.action_id
                      and item.parameter == "search_bounds")
        self.assertEqual(bounds.source, ParameterSource.ENVIRONMENT_OBSERVATION)
        self.assertTrue(any("supported minimum" in item for item in grounded.assumptions))

    def test_numeric_duration_and_missing_duration_have_distinct_provenance(self):
        explicit = ground_command(
            "explore for 2 minutes", task_id="t1", robot_name="robot_1",
            action_servers=ALL, airborne=True,
            environment=CommandEnvironment(current_position=(0.0, 0.0, 1.5)),
        )
        default = ground_command(
            "explore", task_id="t2", robot_name="robot_1",
            action_servers=ALL, airborne=True,
            environment=CommandEnvironment(current_position=(0.0, 0.0, 1.5)),
        )
        explicit_duration = next(item for item in explicit.parameter_grounding
                                 if item.parameter == "time_limit_s")
        default_duration = next(item for item in default.parameter_grounding
                                if item.parameter == "time_limit_s")
        self.assertEqual((explicit.actions[0].time_limit_s, explicit_duration.source),
                         (120.0, ParameterSource.OPERATOR_EXPLICIT))
        self.assertEqual((default.actions[0].time_limit_s, default_duration.source),
                         (30.0, ParameterSource.POLICY_DEFAULT))

    def test_material_ambiguity_asks_one_targeted_question(self):
        with self.assertRaisesRegex(CommandClarificationRequired, "map coordinate"):
            ground_command("fly there", task_id="t1", robot_name="robot_1",
                           action_servers=ALL, airborne=True)
        with self.assertRaisesRegex(CommandClarificationRequired, "between 0.5 and 3"):
            ground_command("take off to 20 meters", task_id="t1", robot_name="robot_1",
                           action_servers=ALL, airborne=False)

    def test_fresh_map_extent_can_tighten_exploration_bounds(self):
        grounded = ground_command(
            "explore for 10 seconds", task_id="t1", robot_name="robot_1",
            action_servers=ALL, airborne=True,
            environment=CommandEnvironment(
                current_position=(0.0, 0.0, 1.5), map_fresh=True,
                map_bounds_xy=(-2.0, 2.0, -2.0, 2.0),
            ),
        )
        xs = [point.x for point in grounded.actions[0].search_bounds]
        self.assertEqual((min(xs), max(xs)), (-1.6, 1.6))
        bounds = next(item for item in grounded.parameter_grounding
                      if item.parameter == "search_bounds")
        self.assertEqual(bounds.source, ParameterSource.ENVIRONMENT_OBSERVATION)


if __name__ == "__main__":
    unittest.main()
