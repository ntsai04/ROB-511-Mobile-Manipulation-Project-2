"""Forward and inverse kinematics for a planar serial arm."""

import math


def forward_position(q: list[float], lengths: list[float]) -> tuple[float, float]:
    """Compute end-effector x/y from relative joint angles."""
    if len(q) != len(lengths):
        raise ValueError("joint and length vectors differ in length")
    x = y = angle = 0.0
    for joint, length in zip(q, lengths):
        angle += joint
        x += length * math.cos(angle)
        y += length * math.sin(angle)
    return x, y


def forward_orientation(q: list[float]) -> float:
    """Compute the final link's absolute orientation."""
    return sum(q)


def solve_ik(
    x: float, y: float, lengths: list[float], phi: float | None = None
) -> list[float]:
    """Solve the closed-form 2-link or 3-link planar IK problem."""
    if len(lengths) not in (2, 3):
        raise ValueError("IK requires a 2- or 3-link arm")
    if len(lengths) == 3:
        phi = 0.0 if phi is None else phi
        x -= lengths[2] * math.cos(phi)
        y -= lengths[2] * math.sin(phi)
    first, second = lengths[0], lengths[1]
    radius = math.hypot(x, y)
    lower, upper = abs(first - second), first + second
    tolerance = 1e-9
    if radius < lower - tolerance or radius > upper + tolerance:
        raise ValueError("target is outside reachable workspace")
    radius = min(upper, max(lower, radius))
    cos_elbow = (radius * radius - first * first - second * second) / (2 * first * second)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.acos(cos_elbow)
    shoulder = math.atan2(y, x) - math.atan2(second * math.sin(elbow), first + second * math.cos(elbow))
    if len(lengths) == 2:
        return [shoulder, elbow]
    return [shoulder, elbow, phi - shoulder - elbow]
