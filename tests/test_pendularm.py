from support import silent, close, rejects, endpoint, two_link_terms, potential, vector_step, checkpoint
import math


@silent
def test_01_euler_coupled_vector():
    vector_step("euler", [1.1, -2.2, .56, -.96])


@silent
def test_02_midpoint_coupled_vector():
    vector_step("midpoint", [1.106, -2.196, .577, -.947])


@silent
def test_03_verlet_predictor_corrector():
    vector_step("verlet", [1.106, -2.196, .5783, -.9471])


@silent
def test_04_rk4_coupled_vector():
    vector_step("rk4", [166051/150000, -43901/20000, 137993/240000, -567109/600000])


@silent
def test_05_euler_ramp_history():
    data = checkpoint("euler", "t")
    h = .125
    for n in range(9):
        close(data["velocities"][n], -.7+h*h*n*(n-1)/2)
        close(data["positions"][n], 1.25-.7*n*h+h**3*n*(n-1)*(n-2)/6)


@silent
def test_06_midpoint_ramp_history():
    data = checkpoint("midpoint", "t")
    for n in range(9):
        t = n*.125
        close(data["velocities"][n], -.7+t*t/2)
        close(data["positions"][n], 1.25-.7*t+t**3/6-n*.125**3/6)


@silent
def test_07_verlet_ramp_history():
    data = checkpoint("verlet", "t")
    for n in range(9):
        t = n*.125
        close(data["velocities"][n], -.7+t*t/2)
        close(data["positions"][n], 1.25-.7*t+t**3/6-n*.125**3/6)


@silent
def test_08_rk4_polynomial_history():
    data = checkpoint("rk4", "t^2 + 3*t - 1")
    for n in range(9):
        t = n*.125
        close(data["velocities"][n], -.7+t**3/3+1.5*t*t-t)
        close(data["positions"][n], 1.25-.7*t+t**4/12+.5*t**3-.5*t*t)


@silent
def test_09_expression_precedence():
    from arm_lib.expr import parse_expression
    for text, t, value in [("2^3^2", 0, 512), ("-2^2", 0, 4), ("10-3-2", 0, 5),
                           ("24/4/2", 0, 3), ("2+3*4", 0, 14), ("(2+3)*4", 0, 20),
                           ("-t + 2*(t-1)^2", .5, 0), (".25 + .01*t", 2, .27)]:
        close(parse_expression(text)(t), value)


@silent
def test_10_expression_functions_and_rejection():
    from arm_lib.expr import parse_expression
    for name, expected in [("sin", math.sin(.4)), ("cos", math.cos(.4)), ("tan", math.tan(.4)),
                           ("exp", math.exp(.4)), ("sqrt", math.sqrt(.4)), ("ln", math.log(.4)), ("abs", .4)]:
        close(parse_expression(name+"(t)")(.4), expected)
    for text in ["", "(t", "t)", "unknown(t)", "1 2", "t@2", "t+", "sin t", "t t"]:
        rejects(parse_expression, text)
    assert math.isnan(parse_expression("sqrt(-1)")(0))


@silent
def test_11_pid_position_velocity_signs():
    from arm_lib.pid import PIDController
    pid = PIDController(3)
    pid.set_gains(kp=[2, 3, 4], kd=[5, 6, 7])
    pid.set_enabled(True)
    close(pid.effort([1, -2, .3], [.2, -.4, .8], [-1, 1, .7], [.5, .1, -.2], .1), [-2.5, 12, -5.4])


@silent
def test_12_pid_integral_dt_and_bounds():
    from arm_lib.pid import PIDController
    pid = PIDController(2)
    pid.set_gains(ki=[2, 3])
    pid.set_enabled(True)
    close(pid.effort([0, 0], [0, 0], [2, -3], [0, 0], .2), [.8, -1.8])
    close(pid.effort([0, 0], [0, 0], [2, -3], [0, 0], .3), [2, -4.5])
    for _ in range(100):
        pid.effort([0, 0], [0, 0], [100, -100], [0, 0], 1)
    close(pid.integral, [pid.integral_limit, -pid.integral_limit])
    close(pid.effort([0, 0], [0, 0], [-1, 1], [0, 0], .1), [2*(pid.integral_limit-.1), 3*(-pid.integral_limit+.1)])


@silent
def test_13_pid_disabled_freezes_integral():
    from arm_lib.pid import PIDController
    pid = PIDController(2)
    pid.set_gains(kp=[3, 4], ki=[1, 2], kd=[5, 6])
    close(pid.effort([1, 2], [3, 4], [9, 8], [7, 6], .5), [0, 0])
    close(pid.integral, [0, 0])
    pid.set_enabled(True)
    pid.effort([0, 0], [0, 0], [1, 2], [0, 0], .5)
    pid.set_enabled(False)
    before = pid.integral[:]
    close(pid.effort([0, 0], [0, 0], [3, 4], [0, 0], .5), [0, 0])
    close(pid.integral, before)


@silent
def test_14_pid_reenable_and_reset():
    from arm_lib.pid import PIDController
    pid = PIDController(2)
    pid.set_enabled(True)
    pid.effort([0, 0], [0, 0], [1, -1], [0, 0], .5)
    pid.set_enabled(True)
    close(pid.integral, [.5, -.5])
    pid.set_enabled(False)
    pid.set_enabled(True)
    close(pid.integral, [0, 0])
    pid.effort([0, 0], [0, 0], [1, -1], [0, 0], .5)
    pid.reset()
    assert pid.enabled
    close(pid.integral, [0, 0])


@silent
def test_15_pid_gain_updates_preserve_history():
    from arm_lib.pid import PIDController
    pid = PIDController(2)
    values = [2, 3]
    pid.set_gains(kp=values, ki=[1, 1], kd=[4, 5])
    values[0] = 99
    pid.set_enabled(True)
    pid.effort([0, 0], [0, 0], [1, 2], [0, 0], .25)
    pid.set_gains(ki=[2, 4])
    close([pid.kp, pid.kd, pid.integral], [[2, 3], [4, 5], [.25, .5]])
    close(pid.effort([0, 0], [0, 0], [0, 0], [0, 0], .1), [.5, 2])
    rejects(pid.set_gains, [1])


@silent
def test_16_mass_matrix_two_link_closed_form():
    from arm_lib.arm_dynamics import ArmDynamics
    from arm_lib.internal import ArmParameters
    masses, lengths = [1.7, .6], [.8, 1.3]
    dynamics = ArmDynamics(ArmParameters(2, 9.81, masses, lengths))
    for q in [[0, 0], [.7, -.9], [-1.2, 2.1]]:
        expected, _, _ = two_link_terms(q, [0, 0], masses, lengths, 9.81)
        matrix = dynamics.mass_matrix(q)
        close(matrix, expected)
        assert matrix[0][0]*matrix[1][1]-matrix[0][1]**2 > 0
    three = ArmDynamics(ArmParameters(3))
    close(three.mass_matrix([0, 0, 0]), [[9, 14/3, 4/3], [14/3, 8/3, 5/6], [4/3, 5/6, 1/3]])


@silent
def test_17_gravity_three_link_potential_gradient():
    from arm_lib.arm_dynamics import ArmDynamics
    from arm_lib.internal import ArmParameters
    masses, lengths, gravity = [1.3, .8, 2], [.6, 1.2, .7], 4.2
    dynamics = ArmDynamics(ArmParameters(3, gravity, masses, lengths))
    q = [.4, -.9, .7]
    expected = []
    for j in range(3):
        plus, minus = q[:], q[:]
        plus[j] += 1e-5
        minus[j] -= 1e-5
        expected.append((potential(plus, masses, lengths, gravity)-potential(minus, masses, lengths, gravity))/2e-5)
    close(dynamics.gravity_load(q), expected, 1e-8)


@silent
def test_18_coriolis_two_link_closed_form():
    from arm_lib.arm_dynamics import ArmDynamics
    from arm_lib.internal import ArmParameters
    masses, lengths = [1.7, .6], [.8, 1.3]
    dynamics = ArmDynamics(ArmParameters(2, 9.81, masses, lengths))
    for q, v in [([.7, -.9], [1.2, -.4]), ([-.8, 1.2], [-.7, .9]), ([.3, .8], [0, 0])]:
        _, expected, _ = two_link_terms(q, v, masses, lengths, 9.81)
        close(dynamics.coriolis_load(q, v), expected, 2e-8)


@silent
def test_19_acceleration_gravity_torque_balance():
    from arm_lib.arm_dynamics import ArmDynamics
    from arm_lib.internal import ArmParameters
    masses, lengths = [1.7, .6], [.8, 1.3]
    dynamics = ArmDynamics(ArmParameters(2, 9.81, masses, lengths))
    for q, v, torque in [([0, 0], [0, 0], [0, 0]), ([.7, -.9], [1.2, -.4], [2, -3]),
                         ([-.8, 1.2], [-.7, .9], [-4, 5])]:
        matrix, coriolis, gravity = two_link_terms(q, v, masses, lengths, 9.81)
        acceleration = dynamics.acceleration(q, v, torque)
        close([sum(matrix[i][j]*acceleration[j] for j in range(2))+coriolis[i]+gravity[i] for i in range(2)], torque, 3e-8)
    matrix, _, gravity = two_link_terms([.4, -.7], [0, 0], masses, lengths, 9.81)
    close(dynamics.acceleration([.4, -.7], [0, 0], gravity), [0, 0])


@silent
def test_20_forward_kinematics_relative_angles():
    from kinematics import forward_position, forward_orientation
    for q, lengths in [([math.pi/2, -math.pi/2], [2, 1]), ([.3, -.8, 1.2], [.7, 1.4, .5]),
                       ([-1.3, .8, -.4], [2, .3, .6])]:
        close(forward_position(q, lengths), endpoint(q, lengths))
        close(forward_orientation(q), sum(q))
    rejects(forward_position, [0], [1, 2])


@silent
def test_21_ik_two_link_unequal_quadrants():
    from kinematics import solve_ik
    lengths = [1.7, .6]
    for target in [[1.5, .5], [-1.5, .5], [-1.5, -.5], [1.5, -.5], [0, 1.5], [0, -1.5]]:
        close(endpoint(solve_ik(*target, lengths), lengths), target)


@silent
def test_22_ik_workspace_boundaries():
    from kinematics import solve_ik
    for target in [[2.5, 0], [-2.5, 0], [0, 1.5], [0, -1.5]]:
        close(endpoint(solve_ik(*target, [2, .5]), [2, .5]), target)
    close(endpoint(solve_ik(0, 0, [1, 1]), [1, 1]), [0, 0])
    for target in [[2.50001, 0], [1.49999, 0], [0, 0]]:
        rejects(solve_ik, *target, [2, .5])


@silent
def test_23_ik_three_link_orientation():
    from kinematics import solve_ik
    lengths = [.9, 1.4, .6]
    for original in [[.4, -.9, .7], [-1.2, 1.6, -.2], [2, .7, -1.1]]:
        target = endpoint(original, lengths)
        phi = sum(original)
        solved = solve_ik(*target, lengths, phi)
        close(endpoint(solved, lengths), target)
        close(sum(solved), phi)
    solved = solve_ik(1.5, .2, lengths)
    close(endpoint(solved, lengths), [1.5, .2])
    close(sum(solved), 0)


@silent
def test_24_simulation_services_and_state():
    from arm_sim_node import ArmSimNode
    from ik_node import IKNode
    arm = ArmSimNode(3)
    success, values, _ = arm.set_params({"gravity": -1, "lengths": [.9, 1.4, .6], "masses": [1, 2, 3]})
    assert not success
    close(values["gravity"], 9.81)
    close(values["lengths"], [.9, 1.4, .6])
    ik = IKNode(arm.parameters)
    assert ik.solve({"x": 1.5, "y": .2, "phi": 0})[0]
    success, values, _ = arm.set_integrator({"method": "bad", "timestep": .01})
    assert not success and values["method"] == "rk4"
    close(values["timestep"], .01)
    success, values, _ = arm.set_gains({"kp": [1, 2, 3], "ki": [-1, 0, 0]})
    assert not success
    close(values["kp"], [1, 2, 3])
    arm.enable_pid({"data": True})
    arm.accept_trajectory({"points": [{"positions": [1, 1, 1]}, {"positions": [.2, -.3, .5]}]})
    close(arm.setpoint.position, [.2, -.3, .5])
    close(arm.setpoint.velocity, [0, 0, 0])
    arm.state.position = [.4, -.2, .1]
    arm.state.velocity = [.1, -.3, .2]
    arm.state.time = 4
    arm.controller.integral = [.3, -.4, .2]
    arm.pause({"data": True})
    arm.step(.01)
    close([arm.state.time, arm.state.position, arm.state.velocity, arm.controller.integral],
          [4, [.4, -.2, .1], [.1, -.3, .2], [.3, -.4, .2]])
    arm.reset({})
    assert arm.controller.enabled and arm.paused
    close([arm.state.position, arm.setpoint.position, arm.controller.integral], [[0, 0, 0]]*3)
    close([arm.state.velocity, arm.state.time], [[0, 0, 0], 0])
    request = dict(function="t", x0=1, xdot0=2, dt=.1, steps=3, integrator="rk4")
    for field, value in [("function", "t+"), ("dt", 0), ("steps", 0), ("integrator", "bad")]:
        assert not arm.integration_step(dict(request, **{field: value}))[0]
    assert arm.integration_step(request)[0]


@silent
def test_25_action_goal_lifecycle():
    from arm_sim_node import ArmSimNode
    from ik_node import IKNode
    from ik_action_node import IKActionNode
    arm = ArmSimNode(2)
    events = []
    action = IKActionNode(IKNode(arm.parameters), arm, lambda t, m: events.append((t, m)))
    ok, values, _ = action.send_goal({"x": 1, "y": 1, "epsilon": .01, "success_hold": .2})
    assert ok
    first = values["goal_id"]
    close(endpoint(arm.setpoint.position, arm.parameters.lengths), [1, 1])
    assert not action.send_goal({"x": 10, "y": 0})[0]
    assert action.active_goal["goal_id"] == first
    assert not action.cancel_goal({"goal_id": "wrong"})[0]
    arm.state.position = arm.setpoint.position[:]
    action.update(1)
    action.update(1.1)
    assert action.active_goal is not None
    action.update(1.21)
    assert action.active_goal is None and action.last_result["outcome"] == "reached"
    action.send_goal({"x": 1.5, "y": .2})
    previous = action.active_goal["goal_id"]
    action.send_goal({"x": .5, "y": 1})
    assert action.last_result["goal_id"] == previous and action.last_result["outcome"] == "preempted"
    assert action.cancel_goal({})[0] and action.active_goal is None
    assert any(topic == "/joint_trajectory" for topic, _ in events)
