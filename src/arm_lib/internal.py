"""Small shared data structures and request validation for Pendularm.

The dataclasses separate configuration (which services may change) from live
state (which only the simulation loop advances).  Service handlers copy
caller-provided vectors before storing them so later mutations of a decoded
request cannot affect the simulation.
"""

from dataclasses import dataclass, field
import math


@dataclass
class ArmParameters:
    """Physical constants shared by dynamics and inverse kinematics."""
    links: int
    gravity: float = 9.81
    masses: list[float] = field(default_factory=list)
    lengths: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Fill omitted mass/length arrays with one uniform unit rod per link."""
        if not self.masses:
            self.masses = [1.0] * self.links
        if not self.lengths:
            self.lengths = [1.0] * self.links


@dataclass
class ArmState:
    """The live generalized coordinates and the simulation clock in seconds."""
    position: list[float]
    velocity: list[float]
    time: float = 0.0


@dataclass
class Setpoint:
    """Held joint-space target taken from the final trajectory point."""
    position: list[float]
    velocity: list[float]


def validate_vector(values: object, length: int, name: str) -> list[float]:
    """Validate and copy a finite numeric vector supplied by a service request.

    Booleans are deliberately rejected even though Python considers them ints:
    accepting ``true`` as a gain or link length would make JSON input silently
    mean something very different from the public service contract.
    """
    if not isinstance(values, list):
        raise ValueError(f"{name} must be an array")
    if len(values) != length:
        raise ValueError(f"{name} must contain {length} values")
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(float(value)) for value in values):
        raise ValueError(f"{name} must contain finite numbers")
    return [float(value) for value in values]
