# Project 2 exam notes

All 25 tests are functions in `tests/test_pendularm.py`; run a single test with
`make test TESTS=test_pendularm.py::test_01_euler_uses_old_velocity_and_start_time`.
The runner prints PASS/FAIL. Test processes emit no stdout or stderr, including
on assertion failure. Preserve this sheet separately: code comments are stripped
in the challenge. All expected values come from public contracts, closed-form
math, or independent geometry; tests never import `src`.

Run the same 25 tests twice, with `ARM_SIM_LINKS=2 make run` and then
`ARM_SIM_LINKS=3 make run`. Each launch needs its own `make test`. The suite
discovers the active count from current parameter lengths. One launch cannot
exercise the other arm count.

| # / function suffix after `test_` | Input and required behavior | Examine if this fails |
| --- | --- | --- |
| 01_euler_uses_old_velocity_and_start_time | Seven steps of t²+3t−1, x=1.25, v=−.4, dt=.125; compare every sample with explicit Euler recurrence | `arm_lib/integrators.py:euler`, service method dispatch |
| 02_midpoint_uses_half_time_and_half_velocity | Same polynomial; midpoint velocity uses f(t+dt/2), position uses v+f(t)dt/2 | `integrators.py:midpoint`; half-stage time/state |
| 03_verlet_averages_endpoint_accelerations | Same polynomial; q uses .5a dt² and v averages endpoint forces | `integrators.py:verlet`; endpoint time and coefficient |
| 04_rk4_matches_polynomial_antiderivatives | Polynomial has exact analytic x=x₀+v₀t+t⁴/12+t³/2−t²/2 and v=v₀+t³/3+1.5t²−t | `integrators.py:rk4`; stage velocities/times, 1:2:2:1 weights |
| 05_expression_associativity_precedence_and_unary | Constant forces from `2^3^2`, `8/4/2`, `-2^2`, grouped arithmetic and decimals | `arm_lib/expr.py` tokenizer, `_power`, `_unary`, `_product`; this course gives unary minus higher priority than power |
| 06_expression_named_functions_and_default_velocity | sin/cos/tan/exp/sqrt/ln/abs combine to force 8; omitted velocity defaults to zero | `expr.py:FUNCTIONS`, function parsing; `arm_sim_node.py:integration_step` defaults |
| 07_malformed_expressions_reject_and_recover | Empty/unbalanced/unknown/trailing/adjacent/invalid-token expressions fail with status; next valid request works | `expr.py` complete-input consumption; gateway exception handling |
| 08_integration_argument_validation_and_recovery | dt≤0, zero/negative/fractional steps, invalid method, nonnumeric x fail; recovery succeeds | `arm_sim_node.py:integration_step`, `_number` |
| 09_checkpoint_service_does_not_change_live_state | While paused run four integrations at x₀=123,v₀=−9; live joint state and clock stay fixed | `integration_step` local state vs `ArmSimNode.state` |
| 10_integrator_partial_updates_queries_and_rejection | Set all methods; dt-only update; invalid method plus valid dt still applies dt; invalid dt preserves old value | `set_integrator`, independent field validation and echo |
| 11_parameter_fields_validate_independently_and_echo | Query/update gravity/masses/lengths; wrong size, zero/negative entries, negative gravity fail independently | `set_params`; `internal.py:validate_vector`; shared parameters |
| 12_pid_gain_fields_validate_independently_and_echo | Query gains; wrong-size/negative arrays preserve old field; valid sibling field applies | `set_gains`, `PIDController.set_gains`, vector validation |
| 13_pause_freezes_clock_state_and_keeps_publishing | Gravity produces motion; pause freezes q/v and header simulation time across continuing messages | `pause`, `step` pause guard, joint-state publication loop |
| 14_reset_clears_motion_old_target_and_pid_integral | Accumulate I toward .4, reset while paused, resume with zero gravity; q/v/effort return zero and enabled remains true | `reset`, `PIDController.reset`, setpoint reset |
| 15_disabled_pid_reports_zero_effort | Nonzero target/gains; disabled output zero, enabled output nonzero, disabling returns zero | `PIDController.effort`, `set_enabled`, live effort publication |
| 16_proportional_term_uses_last_waypoint_per_joint | Nearly immobile massive plant; first waypoint −.9, final alternating per-joint targets; τ=3×final target | `accept_trajectory` final point and relative indexing; PID P sign |
| 17_derivative_term_uses_commanded_velocity_and_defaults | Nearly immobile plant; nonzero v setpoint gives τ=4v; wrong-length velocities become zero | `accept_trajectory` velocity fallback; PID velocity_error sign |
| 18_integral_accumulates_preserves_gains_and_clears_on_enable | Massive plant; effort increase equals simulation elapsed time, doubling Ki retains I, disable/re-enable clears I | `PIDController.effort`, gain updates, `set_enabled`; integration dt |
| 19_pid_converges_to_small_target_without_gravity | .5m/1kg links, Kp=12,Kd=3, dt=.001; alternating ±.18 target within .08rad by sim t>1.8 | PID signs, live integration dispatch, dynamics torque path; allows 20s wall time, within the course's 30s per test |
| 20_zero_gravity_and_zero_torque_preserve_rest | From reset, g=0/PID off leaves all q/v/effort zero | Gravity update/use; stale force or state; integrator |
| 21_gravity_acceleration_matches_coupled_rigid_rods | Unequal masses/lengths, g=4; early v/t matches independently assembled rigid-rod mass matrix and gravity solved by Cholesky, 2% tolerance | `arm_dynamics.py` COM Jacobians, rod inertia m l²/12, downstream coupling, G sign, linear solve |
| 22_ik_roundtrips_quadrants_and_updated_link_lengths | FK-generate targets in four quadrants; IK→independent FK matches x/y and 3-link orientation before/after length update | `kinematics.py:solve_ik`, atan2, cumulative angles; `IKNode` stale lengths |
| 23_ik_reach_boundaries_inner_hole_orientation_and_bad_input | Exact extension and folded 2-link boundary succeed; inner hole, too far, missing/nonnumeric fields and unreachable 3-link wrist fail | Cosine law/clamping, wrist subtraction, input validation |
| 24_action_goal_rejection_cancel_and_preemption | Paused goal; unreachable replacement and wrong cancel preserve it; reachable replacement/cancel publish matching preempted results | `ik_action_node.py` solve-before-preempt ordering, goal IDs, cancel routing, result topic |
| 25_trial_start_skip_stop_clear_status | No-trial skip/stop fail; start emits running shape with zero count; skip then stop clears target/error/desired joints/status | `ik_trial_node.py` lifecycle, `_clear_goal`, action service wiring |

Live state tests pause/reset the arm and isolate settings; they restore parameters,
gains, integrator, enable and pause settings afterward. The pre-test moving pose is
not restored. Run from a clean project launch when investigating a failure.

An entire group failing usually means setup/transport/parameter-query failure.
Use the visualizer's Runtime response panel and the service named in the table
to distinguish setup problems from the invariant itself. Numerical tests compare
all samples, not just the endpoint. The 25 slots group related edge cases; passing
them cannot guarantee detection of every possible black-box mutation.
