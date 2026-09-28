"""Quiet, mutation-oriented checks for the Pendularm implementation.

Each method is intentionally a single pass/fail experiment.  Keep this file
free of print/logging calls: a mutation runner can identify a defect from the
individual test result rather than console output.
"""

from __future__ import annotations

import math
import pathlib
import sys
import unittest


SRC = pathlib.Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from arm_lib.arm_dynamics import ArmDynamics
from arm_lib.expr import parse_expression
from arm_lib.integrators import euler, midpoint, rk4, verlet
from arm_lib.internal import ArmParameters, ArmState
from arm_lib.pid import PIDController
from arm_sim_node import ArmSimNode
from kinematics import forward_orientation, forward_position, solve_ik


class MutationTests(unittest.TestCase):
    """Exactly 25 focused tests, matching the course mutation-test budget."""

    def test_01_euler_uses_old_position_and_velocity(self) -> None:
        """Checks explicit Euler; inspect old-state position/velocity updates if it fails."""
        q, v = euler(lambda _t, _q, _v: [3.0], 0.0, [2.0], [5.0], 0.4)
        self.assertEqual(q, [4.0])
        self.assertEqual(v, [6.2])

    def test_02_midpoint_evaluates_acceleration_at_mid_time(self) -> None:
        """Checks RK2's half-time force sample; inspect midpoint stage time/state construction."""
        q, v = midpoint(lambda t, _q, _v: [t], 0.0, [0.0], [0.0], 0.2)
        self.assertAlmostEqual(q[0], 0.0, places=12)
        self.assertAlmostEqual(v[0], 0.02, places=12)

    def test_03_verlet_predicts_then_corrects_velocity(self) -> None:
        """Checks velocity-Verlet correction; inspect predicted velocity and second acceleration."""
        q, v = verlet(lambda _t, q, _v: [q[0]], 0.0, [1.0], [2.0], 0.1)
        self.assertAlmostEqual(q[0], 1.205, places=12)
        self.assertAlmostEqual(v[0], 2.11025, places=12)

    def test_04_rk4_integrates_time_varying_forcing(self) -> None:
        """Checks all RK4 stages use their times; inspect stage states, times, and 1:2:2:1 weights."""
        q, v = rk4(lambda t, _q, _v: [t], 0.0, [0.0], [0.0], 0.2)
        self.assertAlmostEqual(q[0], 0.2 ** 3 / 6, places=12)
        self.assertAlmostEqual(v[0], 0.2 ** 2 / 2, places=12)

    def test_05_expression_honors_power_associativity_and_unary_precedence(self) -> None:
        """Checks expression precedence; inspect recursive-descent power and unary parsing."""
        value = parse_expression("2^3^2 + -2^2")(0.0)
        self.assertEqual(value, 516.0)

    def test_06_expression_rejects_malformed_source(self) -> None:
        """Checks malformed expression rejection; inspect tokenization and trailing-token validation."""
        for source in ("", "1 2", "sin(t", "unknown(t)", "t @ 2"):
            with self.assertRaises(ValueError):
                parse_expression(source)

    def test_07_integration_service_isolated_from_live_arm(self) -> None:
        """Checks checkpoint isolation; inspect integration_step for writes to live state."""
        arm = ArmSimNode(2)
        arm.state = ArmState([0.3, -0.4], [0.5, -0.6], 7.0)
        ok, values, status = arm.integration_step(
            {"function": "2", "x0": 1.0, "xdot0": -1.0,
             "dt": 0.25, "steps": 2, "integrator": "rk4"}
        )
        self.assertTrue(ok, status)
        self.assertEqual(len(values["times"]), 3)
        self.assertAlmostEqual(values["positions"][-1], 0.75, places=12)
        self.assertEqual(arm.state.position, [0.3, -0.4])
        self.assertEqual(arm.state.velocity, [0.5, -0.6])
        self.assertEqual(arm.state.time, 7.0)

    def test_08_integration_service_rejects_bad_request(self) -> None:
        """Checks invalid checkpoint requests fail cleanly; inspect service input validation."""
        arm = ArmSimNode(2)
        for args in (
            {"function": "t", "x0": 0, "dt": 0, "steps": 1, "integrator": "euler"},
            {"function": "t", "x0": 0, "dt": 0.1, "steps": 0, "integrator": "euler"},
            {"function": "t", "x0": 0, "dt": 0.1, "steps": 1, "integrator": "bogus"},
        ):
            ok, _values, status = arm.integration_step(args)
            self.assertFalse(ok)
            self.assertTrue(status)

    def test_09_pid_disabled_outputs_zero_without_integrating(self) -> None:
        """Checks disabled PID behavior; inspect the enabled guard before integral updates."""
        pid = PIDController(1)
        pid.integral[0] = 3.0
        self.assertEqual(pid.effort([0.0], [0.0], [2.0], [0.0], 0.1), [0.0])
        self.assertEqual(pid.integral, [3.0])

    def test_10_pid_reenable_clears_integral(self) -> None:
        """Checks enable-transition reset; inspect PIDController.set_enabled state handling."""
        pid = PIDController(1)
        pid.set_enabled(True)
        pid.effort([0.0], [0.0], [1.0], [0.0], 0.5)
        self.assertNotEqual(pid.integral, [0.0])
        pid.set_enabled(False)
        pid.set_enabled(True)
        self.assertEqual(pid.integral, [0.0])

    def test_11_pid_derivative_uses_commanded_velocity(self) -> None:
        """Checks D-term velocity error; inspect setpoint-minus-measured velocity calculation."""
        pid = PIDController(1)
        pid.set_gains(kd=[7.0])
        pid.set_enabled(True)
        self.assertEqual(pid.effort([4.0], [-2.0], [4.0], [3.0], 0.1), [35.0])

    def test_12_pid_integral_is_clamped(self) -> None:
        """Checks anti-windup; inspect integral accumulation and its lower/upper clamp."""
        pid = PIDController(1)
        pid.set_gains(ki=[1.0])
        pid.set_enabled(True)
        output = pid.effort([0.0], [0.0], [100.0], [0.0], 1.0)
        self.assertEqual(pid.integral, [pid.integral_limit])
        self.assertEqual(output, [pid.integral_limit])

    def test_13_set_integrator_applies_valid_fields_independently(self) -> None:
        """Checks partial updates; inspect per-field validation in set_integrator."""
        arm = ArmSimNode(2)
        ok, values, status = arm.set_integrator({"method": "bad", "timestep": 0.02})
        self.assertFalse(ok)
        self.assertTrue(status)
        self.assertEqual(values, {"method": "rk4", "timestep": 0.02})

    def test_14_set_params_preserves_invalid_field_and_updates_valid_one(self) -> None:
        """Checks independent parameter updates; inspect set_params error isolation."""
        arm = ArmSimNode(2)
        old_masses = arm.parameters.masses[:]
        ok, values, status = arm.set_params({"gravity": 0.0, "masses": [1.0]})
        self.assertFalse(ok)
        self.assertTrue(status)
        self.assertEqual(values["gravity"], 0.0)
        self.assertEqual(values["masses"], old_masses)

    def test_15_pause_freezes_plant_and_pid_integral(self) -> None:
        """Checks pause semantics; inspect ArmSimNode.step's early return before PID work."""
        arm = ArmSimNode(2)
        arm.enable_pid({"data": True})
        arm.set_gains({"ki": [1.0, 1.0]})
        arm.accept_trajectory({"points": [{"positions": [1.0, 1.0]}]})
        arm.pause({"data": True})
        before = (arm.state.position[:], arm.state.velocity[:], arm.state.time,
                  arm.controller.integral[:])
        arm.step(0.1)
        after = (arm.state.position[:], arm.state.velocity[:], arm.state.time,
                 arm.controller.integral[:])
        self.assertEqual(after, before)

    def test_16_reset_clears_time_setpoint_and_controller_memory(self) -> None:
        """Checks complete reset semantics; inspect arm state, setpoint, and PID reset calls."""
        arm = ArmSimNode(2)
        arm.state = ArmState([1.0, -1.0], [2.0, -2.0], 3.0)
        arm.setpoint.position = [0.3, 0.4]
        arm.controller.integral = [2.0, 3.0]
        ok, values, status = arm.reset({})
        self.assertTrue(ok, status)
        self.assertEqual(values, {"position": [0.0, 0.0], "velocity": [0.0, 0.0]})
        self.assertEqual(arm.state.time, 0.0)
        self.assertEqual(arm.setpoint.position, [0.0, 0.0])
        self.assertEqual(arm.controller.integral, [0.0, 0.0])

    def test_17_trajectory_uses_only_final_point_and_zero_default_velocity(self) -> None:
        """Checks held final setpoint; inspect trajectory last-point and velocity fallback logic."""
        arm = ArmSimNode(2)
        arm.accept_trajectory({"points": [
            {"positions": [8.0, 8.0], "velocities": [7.0, 7.0]},
            {"positions": [1.0, -2.0]},
        ]})
        self.assertEqual(arm.setpoint.position, [1.0, -2.0])
        self.assertEqual(arm.setpoint.velocity, [0.0, 0.0])

    def test_18_joint_state_snapshot_is_copied_and_parallel(self) -> None:
        """Checks publication snapshots; inspect copying and joint-state array construction."""
        arm = ArmSimNode(3)
        snapshot = arm.joint_state_message()
        snapshot["position"][0] = 99.0
        self.assertEqual(arm.state.position[0], 0.0)
        self.assertEqual(snapshot["name"], ["joint1", "joint2", "joint3"])
        self.assertEqual(len(snapshot["velocity"]), 3)
        self.assertEqual(len(snapshot["effort"]), 3)

    def test_19_mass_matrix_is_symmetric_and_positive_diagonal(self) -> None:
        """Checks basic inertia assembly; inspect COM and rotational-Jacobian contributions."""
        mass = ArmDynamics(ArmParameters(2)).mass_matrix([0.4, -0.7])
        self.assertAlmostEqual(mass[0][1], mass[1][0], places=12)
        self.assertGreater(mass[0][0], 0.0)
        self.assertGreater(mass[1][1], 0.0)

    def test_20_zero_gravity_has_zero_static_acceleration(self) -> None:
        """Checks gravity removal; inspect gravity-load use and forward-dynamics signs."""
        parameters = ArmParameters(2, gravity=0.0)
        acceleration = ArmDynamics(parameters).acceleration([0.3, -0.8], [0.0, 0.0], [0.0, 0.0])
        self.assertAlmostEqual(acceleration[0], 0.0, places=12)
        self.assertAlmostEqual(acceleration[1], 0.0, places=12)

    def test_21_gravity_load_matches_static_effort_equilibrium(self) -> None:
        """Checks gravity sign/convention; inspect G(q) and its subtraction in acceleration."""
        dynamics = ArmDynamics(ArmParameters(2))
        q = [0.4, -0.6]
        effort = dynamics.gravity_load(q)
        acceleration = dynamics.acceleration(q, [0.0, 0.0], effort)
        self.assertAlmostEqual(acceleration[0], 0.0, places=10)
        self.assertAlmostEqual(acceleration[1], 0.0, places=10)

    def test_22_coriolis_load_changes_with_joint_velocity(self) -> None:
        """Checks velocity coupling; inspect mass-matrix derivatives and Christoffel assembly."""
        dynamics = ArmDynamics(ArmParameters(2))
        stationary = dynamics.coriolis_load([0.5, -0.9], [0.0, 0.0])
        moving = dynamics.coriolis_load([0.5, -0.9], [1.0, -0.7])
        self.assertEqual(stationary, [0.0, 0.0])
        self.assertTrue(any(abs(value) > 1e-8 for value in moving))

    def test_23_three_link_dynamics_couples_downstream_joint(self) -> None:
        """Checks three-link coupling; inspect downstream COM Jacobian/mass matrix entries."""
        dynamics = ArmDynamics(ArmParameters(3))
        mass = dynamics.mass_matrix([0.2, -0.4, 0.7])
        self.assertNotEqual(mass[0][2], 0.0)
        acceleration = dynamics.acceleration([0.2, -0.4, 0.7], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        self.assertTrue(any(abs(value) > 1e-8 for value in acceleration[1:]))

    def test_24_pid_closed_loop_reaches_a_two_joint_setpoint(self) -> None:
        """Checks end-to-end servo convergence; inspect PID effort, dynamics, or integration."""
        arm = ArmSimNode(2)
        arm.set_params({"gravity": 0.0})
        arm.set_gains({"kp": [25.0, 25.0], "ki": [0.0, 0.0], "kd": [12.0, 12.0]})
        arm.accept_trajectory({"points": [{"positions": [0.4, -0.3]}]})
        arm.enable_pid({"data": True})
        for _ in range(2500):
            arm.step(0.002)
        self.assertLess(abs(arm.state.position[0] - 0.4), 0.02)
        self.assertLess(abs(arm.state.position[1] + 0.3), 0.02)

    def test_25_ik_round_trip_orientation_and_reachability(self) -> None:
        """Checks 3-link IK and rejection; inspect wrist decoupling, angle recovery, or bounds."""
        lengths = [1.0, 0.8, 0.4]
        target = (1.25, 0.35, 0.2)
        q = solve_ik(target[0], target[1], lengths, target[2])
        x, y = forward_position(q, lengths)
        self.assertAlmostEqual(x, target[0], places=10)
        self.assertAlmostEqual(y, target[1], places=10)
        self.assertAlmostEqual(forward_orientation(q), target[2], places=10)
        with self.assertRaises(ValueError):
            solve_ik(10.0, 0.0, [1.0, 1.0])
