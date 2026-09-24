"""Small shared data structures for the Pendularm nodes."""

from dataclasses import dataclass, field
import math


@dataclass
class ArmParameters:
    links: int
    gravity: float = 9.81
    masses: list[float] = field(default_factory=list)
    lengths: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.masses:
            self.masses = [1.0] * self.links
        if not self.lengths:
            self.lengths = [1.0] * self.links


@dataclass
class ArmState:
    position: list[float]
    velocity: list[float]
    time: float = 0.0


@dataclass
class Setpoint:
    position: list[float]
    velocity: list[float]


def validate_vector(values: object, length: int, name: str) -> list[float]:
    """Validate and copy a numeric vector supplied by a service request."""
    if not isinstance(values, list):
        raise ValueError(f"{name} must be an array")
    if len(values) != length:
        raise ValueError(f"{name} must contain {length} values")
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(float(value)) for value in values):
        raise ValueError(f"{name} must contain finite numbers")
    return [float(value) for value in values]
