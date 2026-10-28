# Project 2 - test failure to method/formula

Run `make test` from this project; all 25 tests import source directly and need no running runtime. To isolate a failure: `make test TESTS=test_pendularm.py::test_19_acceleration_gravity_torque_balance`. List full names with `python3 tests/run_tests.py --list`. The runner reports PASS/FAIL; test bodies stay silent. Run `make build` first if many unrelated tests fail. Stop `make run` before `autograde`.

Every code path below is relative to `src/`. Match the numeric test ID first, open the named method, then compare the stated equation or state change. Formula keys I/P/D/K below expand the compact table. Short module names `integrators.py` and `expr.py` mean `arm_lib/integrators.py` and `arm_lib/expr.py`; every `PIDController` method is in `arm_lib/pid.py`, every `ArmDynamics` method in `arm_lib/arm_dynamics.py`, and every `ArmSimNode` method in `arm_sim_node.py`. A failure identifies a check to investigate, not a unique diagnosis; shared helpers or several simultaneous mutations can cause the same pattern.

| ID / exact suffix after `test_` | Open this method first | Specific check when this test fails |
|---|---|---|
| 01 `01_euler_coupled_vector` | `arm_lib/integrators.py:euler` | I1: position uses **old** velocity, velocity uses `a(t,q,v)`. Coupled two-coordinate probe: q'=`[1.1,-2.2]`, v'=`[.56,-.96]`; return fresh lists, leave inputs unchanged. |
| 02 `02_midpoint_coupled_vector` | `arm_lib/integrators.py:midpoint` | I2: use both half-position and half-velocity at `t+h/2`; final position uses `half_v`. q'=`[1.106,-2.196]`, v'=`[.577,-.947]`; no input mutation. |
| 03 `03_verlet_predictor_corrector` | `arm_lib/integrators.py:verlet` | I3: compute `a1` with `new_q` and `predicted_v`; final velocity averages **a0 and a1**. q'=`[1.106,-2.196]`, v'=`[.5783,-.9471]`. |
| 04 `04_rk4_coupled_vector` | `arm_lib/integrators.py:rk4` | I4: correct q/v pair at each of four stages; position weights `v,v2,v3,v4`, velocity weights `a1,a2,a3,a4`, both `1:2:2:1` divided by 6. Check input copying too. |
| 05 `05_euler_ramp_history` | `integrators.py:euler`; `arm_sim_node.py:ArmSimNode.integration_step` | For `a=t`, `t=n*h`: `v_n=v0+h^2*n*(n-1)/2`; `x_n=x0+n*h*v0+h^3*n*(n-1)*(n-2)/6`. If 01 passes, inspect I5 dispatch, initial sample, and time advancement. |
| 06 `06_midpoint_ramp_history` | `integrators.py:midpoint`; `ArmSimNode.integration_step` | For `a=t`: `v_n=v0+t^2/2`; `x_n=x0+v0*t+t^3/6-n*h^3/6`. Check `t+h/2` for acceleration, then I5 histories and live-state isolation. |
| 07 `07_verlet_ramp_history` | `integrators.py:verlet`; `ArmSimNode.integration_step` | Same history equations as 06. If velocity is wrong, inspect `new_v = v+(a0+a1)*h/2`, with `a1` evaluated at `t+h`; practice mutation 17 repeats a0. |
| 08 `08_rk4_polynomial_history` | `integrators.py:rk4`; `ArmSimNode.integration_step` | For `a=t^2+3*t-1`: `v=v0+t^3/3+1.5*t^2-t`; `x=x0+v0*t+t^4/12+.5*t^3-.5*t^2`. Check I4 stages and I5 history/dispatch before blaming dynamics. |
| 09 `09_expression_precedence` | `arm_lib/expr.py:_Parser._sum/_product/_power/_unary`; `_binary` | Sum/product loop left: `10-3-2=5`, `24/4/2=3`. `_power` recurses right: `2^3^2=512`. This course binds unary minus first: `-2^2=4`. Preserve parentheses and decimal literals. |
| 10 `10_expression_functions_and_rejection` | `expr.py:_Parser._tokenize/_atom/parse`; `FUNCTIONS`; `_safe_function` | `_atom` requires function parentheses and a closing `)`, permits only named functions/t. `parse` must consume the end token; reject adjacent/trailing tokens. `ln=math.log`; domain failure `sqrt(-1)` returns NaN rather than raising. |
| 11 `11_pid_position_velocity_signs` | `arm_lib/pid.py:PIDController.effort` | P1: `e=q_target-q`, `ev=v_target-v`; per-joint `kp*e+ki*I+kd*ev`, never reverse either subtraction or share indices. Probe effort=`[-2.5,12,-5.4]`. |
| 12 `12_pid_integral_dt_and_bounds` | `PIDController.effort` | P1: integrate `e*dt` **before** computing effort; clamp I to both `-integral_limit` and `+integral_limit`. Different dt values .2/.3 must accumulate; opposite error must unwind saturation. |
| 13 `13_pid_disabled_freezes_integral` | `PIDController.effort` | Disabled early return must precede the integral update: zero effort and unchanged I, even when I was accumulated earlier. |
| 14 `14_pid_reenable_and_reset` | `PIDController.set_enabled/reset` | P2: clear I only on disabled -> enabled transition; repeated enable preserves I. `reset` clears I but leaves `enabled` unchanged. |
| 15 `15_pid_gain_updates_preserve_history` | `PIDController.set_gains`; `arm_lib/internal.py:validate_vector` | Replace only supplied gain arrays; preserve other gains and I. `validate_vector` returns a **copy** of the correct-length finite list; caller mutations must not alter gains. Wrong-size array rejects. |
| 16 `16_mass_matrix_two_link_closed_form` | `arm_lib/arm_dynamics.py:ArmDynamics.mass_matrix` | D1/D2: COM distance `L/2`, uniform-rod inertia `m*L^2/12`; each link contributes to **all upstream joint pairs**. Check M symmetry and two-link determinant > 0; default straight three-link M=`[[9,14/3,4/3],[14/3,8/3,5/6],[4/3,5/6,1/3]]`. |
| 17 `17_gravity_three_link_potential_gradient` | `ArmDynamics.gravity_load` | D3: cumulative relative angles, downstream masses, COM half-length, **cosine** height derivative. `G_j=dU/dq_j` with `U=sum(m_i*g*y_COM_i)`. This function returns G, not the negative gravity torque. |
| 18 `18_coriolis_two_link_closed_form` | `ArmDynamics.coriolis_load`; then `mass_matrix` | D4: centered derivative denominator `2*delta`; index order `D[k][i][j]+D[j][i][k]-D[i][j][k]`; multiply by `v_j*v_k`. Check the two-link c1/c2 signs in D2. Zero velocities give zero load. |
| 19 `19_acceleration_gravity_torque_balance` | `ArmDynamics.acceleration` | D5: RHS is **effort - coriolis - gravity**. Verify `M*a+c+G=effort`; at rest, effort=G must yield a=0. If 16/17/18 pass, inspect RHS and elimination/pivoting here. Practice mutation 12 adds G. |
| 20 `20_forward_kinematics_relative_angles` | `kinematics.py:forward_position/forward_orientation` | K1: each link uses `theta_i=sum(q[:i+1])`; endpoint sums `L_i*cos(theta_i), L_i*sin(theta_i)`; orientation is `sum(q)`. Reject q/length size mismatch. |
| 21 `21_ik_two_link_unequal_quadrants` | `kinematics.py:solve_ik` | K2: law-of-cosines elbow and `atan2(y,x)-atan2(L2*sin(q2),L1+L2*cos(q2))` shoulder; inspect atan2 argument order/quadrants and unequal lengths. Either elbow branch is valid if FK reaches target. |
| 22 `22_ik_workspace_boundaries` | `kinematics.py:solve_ik` | K2: reachable radius interval `[abs(L1-L2), L1+L2]`, inclusive with `1e-9` tolerance. Reject unreachable input **before** clamping radius/cosine. Equal-link origin succeeds; unequal-link origin lies inside the hole. |
| 23 `23_ik_three_link_orientation` | `kinematics.py:solve_ik` | K3: subtract last-link wrist offset using desired phi, solve first two links, then `q3=phi-q1-q2`. Omitted phi defaults to 0. Check both endpoint and `sum(q)=phi`. |
| 24 `24_simulation_services_and_state` | `arm_sim_node.py:ArmSimNode.set_params/set_integrator/set_gains/accept_trajectory/step/reset/integration_step`; `ik_node.py:IKNode.__init__/solve` | S1 below: partial service updates survive bad sibling fields; IK shares parameters; held setpoint comes from last point; pause returns before clock/PID/integration; reset preserves paused/enabled. Bad checkpoint requests reject and later valid ones recover. |
| 25 `25_action_goal_lifecycle` | `ik_action_node.py:IKActionNode.send_goal/update/cancel_goal/_distance/_finish` | S2 below: solve before preemption; reject mismatched cancel ID; Euclidean FK distance must remain within epsilon for `success_hold`; replacement/cancel outcomes `preempted`, completed outcome `reached`. Use passed simulation time. |

## Integrators and checkpoint time (I)

Let `h=dt`, `v=qdot`, and `a(t,q,v)` be the supplied acceleration callback. All equations are componentwise; return fresh q/v vectors.

- **I1 Euler:** `a0=a(t,q,v)`; `q'=q+h*v`; `v'=v+h*a0`.
- **I2 Midpoint:** `half_q=q+h*v/2`; `half_v=v+h*a0/2`; `ah=a(t+h/2,half_q,half_v)`; `q'=q+h*half_v`; `v'=v+h*ah`.
- **I3 Verlet:** `new_q=q+h*v+h^2*a0/2`; `predicted_v=v+h*a0`; `a1=a(t+h,new_q,predicted_v)`; `new_v=v+h*(a0+a1)/2`. The end acceleration uses predicted velocity because this callback can depend on velocity.
- **I4 RK4:** `a1=a(t,q,v)`; `q2=q+h*v/2`, `v2=v+h*a1/2`, `a2=a(t+h/2,q2,v2)`; `q3=q+h*v2/2`, `v3=v+h*a2/2`, `a3=a(t+h/2,q3,v3)`; `q4=q+h*v3`, `v4=v+h*a3`, `a4=a(t+h,q4,v4)`. Return `q+h*(v+2*v2+2*v3+v4)/6` and `v+h*(a1+2*a2+2*a3+a4)/6`.
- **I5 Checkpoint:** `ArmSimNode.integration_step` parses expression, selects `INTEGRATORS[method]`, starts local `q=[x0], v=[xdot0], time=0`, appends initial sample, integrates at old time, then increments time and appends. `steps=n` gives n+1 samples. Tests 05-08 use `x0=1.25`, `v0=-.7`, `h=.125`, n=0..8. It must not read or overwrite live `arm.state`.

## PID state (P)

**P1:** for each joint, `e=q_target-q`; `I=clamp(I+e*dt,-limit,+limit)`; `ev=v_target-v`; `effort=kp*e+ki*I+kd*ev`. The derivative term uses commanded minus actual velocity, not a finite difference of position error. **P2:** disabled effort is zero without updating I; only a false -> true enable transition clears I; reset clears I without disabling; gain changes leave I intact.

## Dynamics formulas and sign convention (D)

Indices in these formulas are zero-based. Angles are relative, `theta_s=sum(q[0:s+1])`; +x is right, +y is up, and gravity magnitude g is positive. `c` below is already the **vector** returned by `coriolis_load`, so do not multiply it by velocity again.

**D1 General mass matrix:** for link l and upstream joint j<=l, define `d_s=L_s` for s<l, `d_l=L_l/2`. Its COM Jacobian has `Jx_lj=-sum(s=j..l,d_s*sin(theta_s))`, `Jy_lj=sum(s=j..l,d_s*cos(theta_s))`; both are zero for j>l. `I_l=m_l*L_l^2/12`. Then `M_ij=sum(l>=max(i,j), m_l*(Jx_li*Jx_lj+Jy_li*Jy_lj)+I_l)`.

**D2 Two-link check** (use one-based q1/q2, v1/v2, L1/L2 just in this block): `r1=L1/2`, `r2=L2/2`, `I1=m1*L1^2/12`, `I2=m2*L2^2/12`, `b=m2*L1*r2`.

```text
M11=I1+I2+m1*r1^2+m2*(L1^2+r2^2+2*L1*r2*cos(q2))
M12=M21=I2+m2*(r2^2+L1*r2*cos(q2)); M22=I2+m2*r2^2
c1=-b*sin(q2)*(2*v1*v2+v2^2); c2=b*sin(q2)*v1^2
G1=g*((m1*r1+m2*L1)*cos(q1)+m2*r2*cos(q1+q2))
G2=g*m2*r2*cos(q1+q2)
```

**D3 Gravity:** `y_COM_l=sum(s=0..l-1,L_s*sin(theta_s))+(L_l/2)*sin(theta_l)`; `U=sum_l m_l*g*y_COM_l`; `G_j=sum(l>=j,m_l*g*sum(s=j..l,d_s*cos(theta_s)))`. This is a positive potential gradient; `acceleration` subtracts it.

**D4 Coriolis:** source array `D[k][i][j]=dM_ij/dq_k` uses `(M_ij(q+delta*e_k)-M_ij(q-delta*e_k))/(2*delta)`, `delta=1e-6`. `Gamma_ijk=(D[k][i][j]+D[j][i][k]-D[i][j][k])/2`; `c_i=sum_j sum_k Gamma_ijk*v_j*v_k`.

**D5 Solve:** `rhs=effort-c-G`; solve `M*a=rhs`. In `acceleration`, pivot on the largest absolute entry in the remaining column, swap **whole augmented rows**, divide through the RHS column too, eliminate the other rows, and return the final augmented column. If uncertain, check the residual `M*a+c+G-effort`, not a single acceleration sign.

## Geometry and state transitions (K/S)

**K1 FK:** `x=sum_i L_i*cos(theta_i)`; `y=sum_i L_i*sin(theta_i)`; `phi=sum_i q_i`.

**K2 Two-link IK:** `r=hypot(x,y)`, require `abs(L1-L2)-1e-9 <= r <= L1+L2+1e-9`, then clamp r to that interval. `c2=(r^2-L1^2-L2^2)/(2*L1*L2)`; clamp c2 to [-1,1]; `q2=acos(c2)`; `q1=atan2(y,x)-atan2(L2*sin(q2),L1+L2*cos(q2))`. The test accepts any branch that reconstructs the endpoint.

**K3 Three-link IK:** with `phi=0` when omitted, solve K2 at `xw=x-L3*cos(phi)`, `yw=y-L3*sin(phi)`; return `[q1,q2,phi-q1-q2]`.

**S1 Test 24 service/state checklist:** `set_params`, `set_integrator`, and node `set_gains` validate/apply each field independently; response success is false if any field fails, but valid siblings stay applied. Parameters require g>=0, lengths/masses>0; node gains>=0. `IKNode` retains the shared `ArmParameters` object. `accept_trajectory` takes `points[-1]`; missing/invalid velocity becomes zeros. `step` exits immediately if paused. `reset` zeroes q/v/time/setpoint/I/effort without changing paused or enabled. `integration_step` rejects malformed expression, dt<=0, nonpositive steps, unknown integrator before integrating and must work on the next valid call.

**S2 Test 25 action checklist:** `send_goal` validates and solves IK before changing a valid active goal; successful replacement finishes the old goal as `preempted`, publishes `/joint_trajectory`, and updates the arm setpoint locally. `_distance=hypot(FK(q).x-target.x,FK(q).y-target.y)`. In `update(simulation_time)`, distance<=epsilon starts `inside_since`; finish as `reached` only when `simulation_time-inside_since >= success_hold`; leaving the region clears `inside_since`. `cancel_goal` accepts omitted ID or matching ID only; `_finish` stores/publishes result then clears `active_goal`.

## Reading a failure pattern

- **03+07:** inspect Verlet's a1 evaluation and final a0/a1 average first (practice 17).
- **19 with 16-18 passing:** inspect `ArmDynamics.acceleration` subtraction and linear solve (practice 12). If 16, 17, or 18 fails, repair that M/G/c calculation before the combined balance.
- **01-04 pass but 05-08 fail:** inspect `ArmSimNode.integration_step` and expression parsing/time/history; these checkpoints bypass live dynamics and PID.
- **11-15:** inspect PID signs, dt, clamping, and state transitions; these tests bypass integrators/dynamics.
- **20-23:** inspect geometry; if FK20 fails, action25 can fail through `_distance` even when lifecycle code is right. If IK21-23 fail, inspect the solve before action25 orchestration.
- **24/25 only:** inspect S1/S2 and the named node methods; these are direct calls, so a TCP server/port issue does not explain their failure.

For visual execution, run `make run` plus `make visualizer`, then open http://localhost:8002 (forward port 8002 in Docker). Free swing runs with PID disabled; live integrator controls choose the actual runtime method. Source trajectories animate the project's own integration output. To run three links: `ARM_SIM_LINKS=3 make run`. Restart the runtime after source edits.

The baseline passes 25/25. Tolerance normally uses `1e-9*max(1,abs(expected))`, relaxed to 1e-8/2e-8/3e-8 for gravity/Coriolis/balance. Tests check mathematical oracles and state invariants; no finite suite proves every hidden case. TCP transport, stochastic trials, and long-run numerical stability receive less coverage than the core algorithms.
