"""Execution-boundary regressions without ROS, NumPy, or simulator imports."""

from dataclasses import replace
import math
import unittest

from xq_autonomy.ap_execution import (
    APVelocityExecutor, ExecutionConfig, VelocityCommand,
)


def command(**changes):
    return replace(VelocityCommand(1, 10.0, 10.0, 100.0, (0.2, 0.0, 0.0)), **changes)


def step(executor, *, ros=10.0, wall=100.0, **changes):
    inputs = {
        "now_ros_s": ros, "now_monotonic_s": wall,
        "fcu_connected": True, "fcu_armed": True, "fcu_mode": "GUIDED",
        "external_nav_healthy": True, "external_nav_status_monotonic_s": wall,
        "lio_source_stamp_ros_s": ros,
    }
    inputs.update(changes)
    return executor.step(**inputs)


def enabled_executor(config=None):
    executor = APVelocityExecutor(config)
    executor.set_enabled(True)
    return executor


class APExecutionTests(unittest.TestCase):
    def test_disabled_never_publishes_and_reenable_starts_at_zero(self):
        executor = APVelocityExecutor()
        self.assertTrue(executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0))
        self.assertFalse(step(executor).allow_publish)
        executor.set_enabled(True)
        self.assertEqual(step(executor).reason, "no_command")
        self.assertTrue(executor.accept_command(
            command(sequence=2, generated_ros_s=10.1, source_odom_ros_s=10.1, received_monotonic_s=100.1),
            now_ros_s=10.1, now_monotonic_s=100.1,
        ))
        first = step(executor, ros=10.1, wall=100.1)
        # The no-command stop at wall=100 is also an explicit zero origin.
        self.assertLessEqual(math.hypot(*first.velocity_fcu_enu_mps), 0.030000001)
        executor.set_enabled(False)
        self.assertFalse(step(executor, ros=10.11, wall=100.11).allow_publish)
        executor.set_enabled(True)
        self.assertTrue(executor.accept_command(
            command(sequence=3, generated_ros_s=10.2, source_odom_ros_s=10.2, received_monotonic_s=100.2),
            now_ros_s=10.2, now_monotonic_s=100.2,
        ))
        self.assertEqual(step(executor, ros=10.2, wall=100.2).velocity_fcu_enu_mps, (0.0, 0.0, 0.0))

    def test_duplicate_receipts_do_not_feed_watchdog_when_ros_clock_freezes(self):
        executor = enabled_executor()
        self.assertTrue(executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0))
        step(executor)
        self.assertGreater(step(executor, wall=100.1).velocity_fcu_enu_mps[0], 0.0)
        self.assertFalse(executor.accept_command(
            command(received_monotonic_s=100.3), now_ros_s=10.0, now_monotonic_s=100.3,
        ))
        self.assertEqual(executor.last_rejection_reason, "sequence_not_increasing")
        stopped = step(executor, wall=100.4)
        self.assertTrue(stopped.allow_publish)
        self.assertEqual(stopped.reason, "command_wall_stale")
        self.assertEqual(stopped.velocity_fcu_enu_mps, (0.0, 0.0, 0.0))

    def test_invalid_future_and_out_of_order_commands_are_rejected(self):
        invalid = (
            {"sequence": True}, {"velocity_map_mps": (math.nan, 0.0, 0.0)},
            {"generated_ros_s": math.inf}, {"source_odom_ros_s": math.nan},
            {"received_monotonic_s": math.inf}, {"generated_ros_s": 10.01},
            {"source_odom_ros_s": 10.01}, {"received_monotonic_s": 100.01},
        )
        for changes in invalid:
            with self.subTest(changes=changes):
                executor = enabled_executor()
                self.assertFalse(executor.accept_command(
                    command(**changes), now_ros_s=10.0, now_monotonic_s=100.0,
                ))
                self.assertEqual(step(executor).reason, "no_command")
        executor = enabled_executor()
        self.assertTrue(executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0))
        for changes, expected in (
            ({"sequence": 0}, "sequence_not_increasing"),
            ({"sequence": 2, "generated_ros_s": 9.99}, "generation_out_of_order"),
            ({"sequence": 2, "generated_ros_s": 10.1, "source_odom_ros_s": 9.99}, "source_odom_out_of_order"),
        ):
            self.assertFalse(executor.accept_command(
                command(**changes), now_ros_s=10.1, now_monotonic_s=100.1,
            ))
            self.assertEqual(executor.last_rejection_reason, expected)

    def test_new_sequence_can_share_generation_but_replay_does_not_refresh_receipt(self):
        executor = enabled_executor()
        self.assertTrue(executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0))
        step(executor)
        same_tick = command(sequence=2, received_monotonic_s=100.1, velocity_map_mps=(0.0, 0.2, 0.0))
        self.assertTrue(executor.accept_command(same_tick, now_ros_s=10.0, now_monotonic_s=100.1))
        output = step(executor, wall=100.1)
        self.assertGreater(output.velocity_fcu_enu_mps[1], 0.0)
        self.assertEqual(output.velocity_fcu_enu_mps[0], 0.0)
        self.assertFalse(executor.accept_command(
            replace(same_tick, received_monotonic_s=100.4),
            now_ros_s=10.0, now_monotonic_s=100.4,
        ))
        self.assertEqual(executor.last_rejection_reason, "sequence_not_increasing")
        stopped = step(executor, wall=100.5)
        self.assertTrue(stopped.allow_publish)
        self.assertEqual(stopped.reason, "command_wall_stale")
        self.assertEqual(stopped.velocity_fcu_enu_mps, (0.0, 0.0, 0.0))

    def test_clock_rollbacks_latch_and_do_not_revive_old_commands(self):
        for ros, wall, reason in (
            (9.0, 100.1, "ros_clock_rollback"),
            (10.1, 99.0, "monotonic_clock_rollback"),
        ):
            with self.subTest(reason=reason):
                executor = enabled_executor()
                executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0)
                step(executor)
                failed = step(executor, ros=ros, wall=wall)
                self.assertFalse(failed.allow_publish)
                self.assertEqual(failed.reason, reason)
                executor.set_enabled(False)
                executor.set_enabled(True)
                self.assertFalse(executor.accept_command(
                    command(sequence=2, generated_ros_s=10.2, source_odom_ros_s=10.2, received_monotonic_s=100.2),
                    now_ros_s=10.2, now_monotonic_s=100.2,
                ))
                self.assertEqual(step(executor, ros=10.2, wall=100.2).reason, reason)

    def test_fixed_yaw_rotates_xy_and_preserves_vertical_component(self):
        executor = enabled_executor(ExecutionConfig(
            max_speed_mps=2.0, max_acceleration_mps2=10.0,
            map_to_fcu_enu_yaw_rad=math.pi / 2.0,
        ))
        executor.accept_command(
            command(velocity_map_mps=(1.0, 0.0, 0.5)), now_ros_s=10.0, now_monotonic_s=100.0,
        )
        self.assertEqual(step(executor).velocity_fcu_enu_mps, (0.0, 0.0, 0.0))
        output = step(executor, ros=10.2, wall=100.2).velocity_fcu_enu_mps
        self.assertAlmostEqual(output[0], 0.0)
        self.assertAlmostEqual(output[1], 1.0)
        self.assertAlmostEqual(output[2], 0.5)

    def test_normal_speed_and_acceleration_limits_use_vector_norm_and_wall_time(self):
        executor = enabled_executor()
        executor.accept_command(
            command(velocity_map_mps=(1.0, 1.0, 1.0)), now_ros_s=10.0, now_monotonic_s=100.0,
        )
        self.assertEqual(step(executor).velocity_fcu_enu_mps, (0.0, 0.0, 0.0))
        previous = (0.0, 0.0, 0.0)
        for elapsed in (0.1, 0.2, 0.3):
            # Identical enable notifications must not reset the ramp.
            executor.set_enabled(True)
            current = step(executor, wall=100.0 + elapsed).velocity_fcu_enu_mps
            self.assertAlmostEqual(math.dist(current, previous), 0.03)
            self.assertLessEqual(math.hypot(*current), 0.2)
            previous = current

    def test_stale_command_stops_only_while_localization_is_healthy(self):
        executor = enabled_executor()
        executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0)
        step(executor)
        step(executor, ros=10.1, wall=100.1)
        stopped = step(executor, ros=10.4, wall=100.4)
        self.assertTrue(stopped.allow_publish)
        self.assertEqual(stopped.velocity_fcu_enu_mps, (0.0, 0.0, 0.0))
        self.assertEqual(stopped.reason, "command_ros_stale")
        failed = step(executor, ros=10.5, wall=100.5, lio_source_stamp_ros_s=9.0)
        self.assertFalse(failed.allow_publish)
        self.assertEqual(failed.reason, "lio_source_stale")

    def test_fresh_command_after_unobserved_timeout_restarts_ramp_at_zero(self):
        executor = enabled_executor()
        executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0)
        step(executor)
        self.assertGreater(step(executor, ros=10.1, wall=100.1).velocity_fcu_enu_mps[0], 0.0)
        # Simulate a blocked timer: a fresh command callback runs first when
        # execution resumes, so there has been no explicit stale step.
        self.assertTrue(executor.accept_command(
            command(sequence=2, generated_ros_s=11.0, source_odom_ros_s=11.0, received_monotonic_s=101.0),
            now_ros_s=11.0, now_monotonic_s=101.0,
        ))
        self.assertEqual(step(executor, ros=11.0, wall=101.0).velocity_fcu_enu_mps, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(step(executor, ros=11.1, wall=101.1).velocity_fcu_enu_mps[0], 0.03)

    def test_speed_limit_applies_to_whole_3d_vector(self):
        executor = enabled_executor(ExecutionConfig(max_acceleration_mps2=10.0))
        executor.accept_command(
            command(velocity_map_mps=(1.0, 1.0, 1.0)), now_ros_s=10.0, now_monotonic_s=100.0,
        )
        step(executor)
        output = step(executor, ros=10.1, wall=100.1).velocity_fcu_enu_mps
        self.assertAlmostEqual(math.hypot(*output), 0.2)
        for component in output:
            self.assertAlmostEqual(component, 0.2 / math.sqrt(3.0))

    def test_flight_and_external_nav_interlocks_block_all_setpoints(self):
        for changes, reason in (
            ({"fcu_connected": False}, "fcu_disconnected"),
            ({"fcu_armed": False}, "fcu_disarmed"),
            ({"fcu_mode": "LAND"}, "fcu_not_guided"),
            ({"external_nav_healthy": False}, "external_nav_unhealthy"),
            ({"external_nav_status_monotonic_s": 97.9}, "external_nav_status_stale"),
            ({"lio_source_stamp_ros_s": 10.01}, "lio_stamp_future"),
        ):
            with self.subTest(reason=reason):
                executor = enabled_executor()
                executor.accept_command(command(), now_ros_s=10.0, now_monotonic_s=100.0)
                output = step(executor, **changes)
                self.assertFalse(output.allow_publish)
                self.assertEqual(output.reason, reason)


if __name__ == "__main__":
    unittest.main()
