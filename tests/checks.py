"""Quiet standard-library test utilities; only public TCP interfaces are used."""
import contextlib
import functools
import io
import math
import time

from rosbridge_client import Rosbridge


def silent(function):
    @functools.wraps(function)
    def wrapped():
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                function()
            except BaseException:
                raise SystemExit(1) from None
    return wrapped


def call(ros, name, args=None, success=True):
    reply = ros.call_service(name, {} if args is None else args, timeout=2)
    assert reply.get('op') == 'service_response'
    assert reply.get('result') is success
    assert isinstance(reply.get('values'), dict)
    assert isinstance(reply.get('status'), str)
    if not success:
        assert reply['status']
    return reply['values']


def close(actual, expected, tolerance=1e-8):
    assert math.isfinite(actual) and abs(actual - expected) <= tolerance * max(1, abs(expected))


def vector(actual, expected, tolerance=1e-8):
    assert len(actual) == len(expected)
    for a, b in zip(actual, expected):
        close(a, b, tolerance)


@contextlib.contextmanager
def session():
    with Rosbridge(timeout=2) as ros:
        params = call(ros, '/arm_sim/set_params')
        gains = call(ros, '/pid_controller/set_gains')
        integrator = call(ros, '/arm_sim/set_integrator')
        paused = call(ros, '/arm_sim/pause')['paused']
        enabled = call(ros, '/pid_controller/enable')['enabled']
        n = len(params['lengths'])
        assert n in (2, 3)
        call(ros, '/arm_sim/pause', {'data': True})
        ros.call_service('/ik_trial/stop', {}, timeout=2)
        ros.call_service('/ik_action/cancel_goal', {}, timeout=2)
        call(ros, '/pid_controller/enable', {'data': False})
        call(ros, '/arm_sim/set_integrator', {'method': 'rk4', 'timestep': 0.005})
        call(ros, '/arm_sim/set_params', {'gravity': 0, 'masses': [1.0]*n, 'lengths': [1.0]*n})
        call(ros, '/pid_controller/set_gains', {'kp': [0.0]*n, 'ki': [0.0]*n, 'kd': [0.0]*n})
        call(ros, '/arm_sim/reset')
        try:
            yield ros, n
        finally:
            call(ros, '/arm_sim/pause', {'data': True})
            ros.call_service('/ik_trial/stop', {}, timeout=2)
            ros.call_service('/ik_action/cancel_goal', {}, timeout=2)
            call(ros, '/arm_sim/set_params', params)
            call(ros, '/pid_controller/set_gains', gains)
            call(ros, '/arm_sim/set_integrator', integrator)
            call(ros, '/arm_sim/reset')
            call(ros, '/pid_controller/enable', {'data': enabled})
            call(ros, '/arm_sim/pause', {'data': paused})


def stamp(message):
    value = message['header']['stamp']
    return value['sec'] + value['nanosec'] / 1e9


def subscribe(ros):
    ros.subscribe('/joint_states', 'sensor_msgs/JointState', timeout=2)


def state(ros, n, predicate=lambda s: True, timeout=3):
    value = ros.next_message('/joint_states', timeout=timeout, match=predicate)
    assert len(value['name']) == n and len(set(value['name'])) == n
    for field in ('position', 'velocity', 'effort'):
        assert len(value[field]) == n and all(math.isfinite(v) for v in value[field])
    stamp(value)
    return value


def trajectory(ros, n, positions, velocities=None, first=None):
    ros.advertise('/joint_trajectory', 'trajectory_msgs/JointTrajectory')
    point = {'positions': positions}
    if velocities is not None:
        point['velocities'] = velocities
    points = ([{'positions': first, 'velocities': [0]*n}] if first is not None else []) + [point]
    ros.publish('/joint_trajectory', {'joint_names': [f'joint{i+1}' for i in range(n)], 'points': points})
    # Same connection serializes publication before this query/acknowledgement.
    call(ros, '/arm_sim/pause')


def frozen_plant(ros, n, kp=0, ki=0, kd=0):
    call(ros, '/arm_sim/set_params', {'gravity': 0, 'masses': [1e10]*n})
    call(ros, '/pid_controller/set_gains', {'kp': [kp]*n, 'ki': [ki]*n, 'kd': [kd]*n})
    call(ros, '/pid_controller/enable', {'data': True})
    subscribe(ros)


def integrate(ros, method, expression='t^2 + 3*t - 1', x0=1.25, v0=-0.4, dt=0.125, steps=7):
    result = call(ros, '/arm_sim/integration_step', {'function': expression, 'x0': x0, 'xdot0': v0,
                  'dt': dt, 'steps': steps, 'integrator': method})
    for field in ('times', 'positions', 'velocities'):
        assert len(result[field]) == steps + 1
    vector(result['times'], [i*dt for i in range(steps+1)])
    close(result['positions'][0], x0)
    close(result['velocities'][0], v0)
    return result


def fk(q, lengths):
    angle = x = y = 0.0
    for value, length in zip(q, lengths):
        angle += value
        x += length * math.cos(angle)
        y += length * math.sin(angle)
    return x, y, angle


def ik_roundtrip(ros, q, lengths):
    x, y, phi = fk(q, lengths)
    actual = call(ros, '/ik/solve', {'x': x, 'y': y, 'phi': phi})['positions']
    assert len(actual) == len(q)
    ax, ay, ap = fk(actual, lengths)
    close(ax, x)
    close(ay, y)
    if len(q) == 3:
        close(math.sin(ap-phi), 0)
        close(math.cos(ap-phi), 1)
