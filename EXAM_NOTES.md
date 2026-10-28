# Project 2 — Pendularm mutation tests

Run `make test` from this project. Exactly 25 independent tests use the course harness; each test emits no output and reports only pass/fail through the runner. These tests import source directly and **do not require `make run`**. The runner's “nothing is listening” note is harmless for this suite. To run one: `make test TESTS=test_pendularm.py::test_19_acceleration_gravity_torque_balance`. List names: `python3 tests/run_tests.py --list`. Stop a running project before `autograde`.

Run `make visualizer`, then open **http://localhost:8002**. The visualizer runs the project algorithms and displays their computed arm geometry, integrator traces, PID effort, and dynamics state. Source views work immediately. For live snapshots and servo controls, run `make run` in another terminal. Launch three links with `ARM_SIM_LINKS=3 make run`. In Docker, forward port 8002 to the host (VS Code Ports panel). Every browser asset is local; no network or packages are needed.

The names below are suffixes of `test_pendularm.py::test_`. Paths are relative to `src/`.

| ID and test name | Input / expected invariant | First code to inspect |
| --- | --- | --- |
| 01 `01_euler_coupled_vector` | Two coupled coordinates, nonzero t/q/v; explicit Euler gives `[1.1,-2.2]`, `[.56,-.96]`; inputs unchanged. | `arm_lib/integrators.py:euler`; use old velocity for position. |
| 02 `02_midpoint_coupled_vector` | Same coupled system; q=`[1.106,-2.196]`, v=`[.577,-.947]`. | `midpoint`; half-time, half-position, half-velocity. |
| 03 `03_verlet_predictor_corrector` | Same system; q=`[1.106,-2.196]`, v=`[.5783,-.9471]`. | `verlet`; position half-acceleration, predicted velocity, average a0/a1. |
| 04 `04_rk4_coupled_vector` | Exact rational one-step oracle for coupled position/velocity; four stages and 1:2:2:1 weights. | `rk4`; stage q/v pairing and time increments. |
| 05 `05_euler_ramp_history` | x″=t, eight dt=.125 steps. v=v0+h²n(n−1)/2; x=x0+nv0h+h³n(n−1)(n−2)/6. | `euler`; checkpoint initial sample, step/time indexing. |
| 06 `06_midpoint_ramp_history` | x″=t; v=v0+t²/2; x=x0+v0t+t³/6−nh³/6. Full history and no live-state changes. | `midpoint`; midpoint acceleration/time; `integration_step`. |
| 07 `07_verlet_ramp_history` | Same ramp oracle as 06; velocity must use end acceleration. | `verlet`; a0/a1 average. Detects practice mutation 17. |
| 08 `08_rk4_polynomial_history` | x″=t²+3t−1; exact polynomial x/v at all samples. | `rk4`; stage weights; checkpoint dispatch and time. |
| 09 `09_expression_precedence` | Right-associative `2^3^2=512`, left-associative subtraction/division, parentheses/decimals. Course unary rule gives `-2^2=4`. | `arm_lib/expr.py:_sum/_product/_power/_unary`. |
| 10 `10_expression_functions_and_rejection` | Seven named functions agree with math; empty/unbalanced/unknown/adjacent/trailing inputs rejected; sqrt(−1) is NaN. | `expr.py`; tokenization, closing delimiters, end-of-input check. |
| 11 `11_pid_position_velocity_signs` | Three joints with unequal Kp/Kd; output `[-2.5,12,-5.4]`. | `arm_lib/pid.py:effort`; desired−actual signs and joint indexing. |
| 12 `12_pid_integral_dt_and_bounds` | Unequal dt=.2,.3 accumulates I; positive/negative saturation; reverse error unwinds. | `effort`; error×dt before effort; clamp both signs. |
| 13 `13_pid_disabled_freezes_integral` | Disabled output zero and integral unchanged, including after previously enabled accumulation. | `effort` disabled early return. |
| 14 `14_pid_reenable_and_reset` | Repeated enable preserves I; disable→enable clears I; reset clears I and retains enabled state. | `PIDController.set_enabled/reset`. |
| 15 `15_pid_gain_updates_preserve_history` | Partial Ki update preserves Kp/Kd/I; gain arrays copied; wrong-size gains rejected. | `set_gains`; `internal.validate_vector`; avoid hidden reset/aliasing. |
| 16 `16_mass_matrix_two_link_closed_form` | Unequal masses/lengths, straight and bent poses; two-link M matches uniform-rod formula; three-link straight M has exact rational entries. | `arm_lib/arm_dynamics.py:mass_matrix`; COM half-length, rod inertia mL²/12, upstream contributions. |
| 17 `17_gravity_three_link_potential_gradient` | Three links, unequal parameters; G matches finite difference of independent gravitational potential. | `gravity_load`; cumulative relative angles and downstream mass contributions. |
| 18 `18_coriolis_two_link_closed_form` | Nonzero positive/negative velocities; C1=−b sin(q2)(2v1v2+v2²), C2=b sin(q2)v1². | `coriolis_load`; Christoffel derivative indices/signs. |
| 19 `19_acceleration_gravity_torque_balance` | Independent M/C/G verify M·a+C+G=τ; τ=G at rest yields a=0. | `acceleration`; RHS **τ−C−G**, solver elimination. Detects practice mutation 12. |
| 20 `20_forward_kinematics_relative_angles` | Two/three links with mixed relative angles and lengths; FK matches sum of absolute-angle link vectors. | `kinematics.forward_position/forward_orientation`; accumulate angles. |
| 21 `21_ik_two_link_unequal_quadrants` | Unequal links, targets in four quadrants and vertical axes; independent FK of IK solution reaches target. | `solve_ik`; atan2 quadrant, law of cosines, shoulder subtraction. |
| 22 `22_ik_workspace_boundaries` | Inner/outer radii accepted; equal-link origin accepted; just inside hole/outside reach rejected. | `solve_ik`; abs(L1−L2), L1+L2, tolerance/clamping. |
| 23 `23_ik_three_link_orientation` | Several poses/orientations; solved FK reaches target, sum(q)=φ; omitted φ defaults zero. | `solve_ik`; subtract last-link wrist offset; q3=φ−q1−q2. |
| 24 `24_simulation_services_and_state` | Mixed valid/invalid fields apply independently; shared IK lengths; final trajectory point wins; pause freezes clock/I; reset clears target/I and preserves enabled/paused. Bad checkpoints rejected, then valid call works. | `arm_sim_node.py:set_params/set_integrator/set_gains/accept_trajectory/step/reset/integration_step`; `ik_node.py`. |
| 25 `25_action_goal_lifecycle` | Valid goal commands arm; unreachable goal preserves active goal; wrong cancel ID rejected; dwell required; replacement/cancel produce preempted. | `ik_action_node.py:send_goal/update/cancel_goal/_finish`; compare simulation time. |

Failure patterns: 03+07 strongly suggest Verlet's final velocity. Only 19 suggests acceleration assembly/solver; 16/17/18 identify its inputs. A block of 01–08 suggests integrator dispatch or checkpoint time bookkeeping. 11–15 isolate PID state and signs. 20–23 isolate geometry. 24/25 cover orchestration. Many unrelated failures can mean an import/syntax or shared helper error; run `make build` first.

Oracles use formulas, rational one-step values, physical balance, and independent geometric reconstruction, rather than saved output from the passing project. Tests accept any IK elbow branch that reaches the target. Default numerical tolerance is relative/absolute 1e−9, relaxed to 1e−8–3e−8 for finite-difference dynamics. Every direct node test creates fresh state, so order does not matter. No runtime timing or convergence threshold is used. No 25-test suite can guarantee detection of every possible mutation; the transport and stochastic trial sampler receive less coverage than the core algorithms.
