"""Second-order numerical integration methods used by the simulator.

All methods accept the same acceleration callback, ``qddot = f(t, q, qdot)``,
and return fresh position/velocity lists after one timestep.  Keeping that
one interface is what lets the checkpoint service and live arm use exactly
the same implementations.
"""

from collections.abc import Callable

Acceleration = Callable[[float, list[float], list[float]], list[float]]


def euler(
    acceleration: Acceleration, t: float, q: list[float], qdot: list[float], dt: float
) -> tuple[list[float], list[float]]:
    """Advance one step with explicit Euler."""
    a = acceleration(t, q, qdot)
    return ([x + v * dt for x, v in zip(q, qdot)],
            [v + x * dt for v, x in zip(qdot, a)])


def midpoint(
    acceleration: Acceleration, t: float, q: list[float], qdot: list[float], dt: float
) -> tuple[list[float], list[float]]:
    """Advance one step with midpoint/RK2 integration.

    First predict the state halfway through the step, then evaluate the force
    at that halfway time and state for the full update.
    """
    a0 = acceleration(t, q, qdot)
    half_q = [x + v * dt / 2 for x, v in zip(q, qdot)]
    half_v = [v + a * dt / 2 for v, a in zip(qdot, a0)]
    ah = acceleration(t + dt / 2, half_q, half_v)
    return ([x + v * dt for x, v in zip(q, half_v)],
            [v + a * dt for v, a in zip(qdot, ah)])


def verlet(
    acceleration: Acceleration, t: float, q: list[float], qdot: list[float], dt: float
) -> tuple[list[float], list[float]]:
    """Advance one step with predictor-corrector velocity Verlet.

    The provisional velocity matters here because arm acceleration can depend
    on velocity through Coriolis terms, not only on position.
    """
    a0 = acceleration(t, q, qdot)
    new_q = [x + v * dt + a * dt * dt / 2 for x, v, a in zip(q, qdot, a0)]
    predicted_v = [v + a * dt for v, a in zip(qdot, a0)]
    a1 = acceleration(t + dt, new_q, predicted_v)
    new_v = [v + (a + b) * dt / 2 for v, a, b in zip(qdot, a0, a1)]
    return new_q, new_v


def rk4(
    acceleration: Acceleration, t: float, q: list[float], qdot: list[float], dt: float
) -> tuple[list[float], list[float]]:
    """Advance one step with classical fourth-order Runge-Kutta."""
    # The coupled first-order state is (q, qdot): q' = qdot, qdot' = a(...).
    a1 = acceleration(t, q, qdot)
    q2 = [x + v * dt / 2 for x, v in zip(q, qdot)]
    v2 = [v + a * dt / 2 for v, a in zip(qdot, a1)]
    a2 = acceleration(t + dt / 2, q2, v2)
    q3 = [x + v * dt / 2 for x, v in zip(q, v2)]
    v3 = [v + a * dt / 2 for v, a in zip(qdot, a2)]
    a3 = acceleration(t + dt / 2, q3, v3)
    q4 = [x + v * dt for x, v in zip(q, v3)]
    v4 = [v + a * dt for v, a in zip(qdot, a3)]
    a4 = acceleration(t + dt, q4, v4)
    new_q = [x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
             for x, k1, k2, k3, k4 in zip(q, qdot, v2, v3, v4)]
    new_v = [x + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
             for x, k1, k2, k3, k4 in zip(qdot, a1, a2, a3, a4)]
    return new_q, new_v


INTEGRATORS = {
    "euler": euler,
    "midpoint": midpoint,
    "verlet": verlet,
    "rk4": rk4,
}
