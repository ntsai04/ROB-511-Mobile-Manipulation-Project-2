"""Quiet assertion helpers and independent numerical oracles."""

import contextlib
import functools
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(os.environ.get("AUTOROB_SOURCE", ROOT / "src")).resolve()))
sys.excepthook = lambda *_args: None


def silent(function):
    @functools.wraps(function)
    def run():
        with open(os.devnull, "w") as sink:
            with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
                try:
                    function()
                except BaseException:
                    raise SystemExit(1) from None
    return run


def close(actual, expected, tolerance=1e-9):
    if isinstance(expected, (tuple, list)):
        assert len(actual) == len(expected)
        for a, b in zip(actual, expected):
            close(a, b, tolerance)
    else:
        assert math.isfinite(actual) and abs(actual - expected) <= tolerance * max(1, abs(expected))


def rejects(function, *args):
    try:
        function(*args)
    except (ValueError, TypeError):
        return
    raise AssertionError


def endpoint(q, lengths):
    angles = [sum(q[:i + 1]) for i in range(len(q))]
    return [sum(length * math.cos(a) for length, a in zip(lengths, angles)),
            sum(length * math.sin(a) for length, a in zip(lengths, angles))]


def two_link_terms(q, v, masses, lengths, gravity):
    m1, m2 = masses
    l1, l2 = lengths
    c1, c2 = l1 / 2, l2 / 2
    i1, i2 = m1 * l1*l1 / 12, m2 * l2*l2 / 12
    b = m2*l1*c2
    m11 = i1+i2+m1*c1*c1+m2*(l1*l1+c2*c2+2*l1*c2*math.cos(q[1]))
    m12 = i2+m2*(c2*c2+l1*c2*math.cos(q[1]))
    m22 = i2+m2*c2*c2
    coriolis = [-b*math.sin(q[1])*(2*v[0]*v[1]+v[1]*v[1]), b*math.sin(q[1])*v[0]*v[0]]
    load = [gravity*((m1*c1+m2*l1)*math.cos(q[0])+m2*c2*math.cos(sum(q))),
            gravity*m2*c2*math.cos(sum(q))]
    return [[m11, m12], [m12, m22]], coriolis, load


def potential(q, masses, lengths, gravity):
    total = 0.0
    for i, mass in enumerate(masses):
        y = sum(lengths[j]*math.sin(sum(q[:j+1])) for j in range(i))
        y += lengths[i]/2*math.sin(sum(q[:i+1]))
        total += mass*gravity*y
    return total


def vector_step(method, expected):
    from arm_lib.integrators import INTEGRATORS
    q, v = [1.0, -2.0], [.5, -1.0]
    def acceleration(t, position, velocity):
        return [t+2*position[0]-3*velocity[0]+position[1]/4,
                -t+position[0]/2-position[1]+2*velocity[1]]
    result = INTEGRATORS[method](acceleration, .3, q, v, .2)
    close(result[0] + result[1], expected)
    close(q, [1, -2])
    close(v, [.5, -1])
    assert result[0] is not q and result[1] is not v


def checkpoint(method, expression, n=8, h=.125, x0=1.25, v0=-.7):
    from arm_sim_node import ArmSimNode
    arm = ArmSimNode(2)
    arm.state.position = [.3, -.8]
    arm.state.velocity = [.2, -.1]
    arm.state.time = 4.0
    success, values, _ = arm.integration_step(dict(function=expression, integrator=method,
                                                 steps=n, dt=h, x0=x0, xdot0=v0))
    assert success and all(len(values[k]) == n+1 for k in ("times", "positions", "velocities"))
    close(values["times"], [i*h for i in range(n+1)])
    close([values["positions"][0], values["velocities"][0]], [x0, v0])
    close([arm.state.position, arm.state.velocity, arm.state.time], [[.3, -.8], [.2, -.1], 4])
    return values
