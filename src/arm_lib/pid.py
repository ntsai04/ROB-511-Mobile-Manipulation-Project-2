"""Per-joint PID controller with bounded integral state."""

from .internal import validate_vector


class PIDController:
    def __init__(self, links: int) -> None:
        self.kp = [0.0] * links
        self.ki = [0.0] * links
        self.kd = [0.0] * links
        self.integral = [0.0] * links
        self.integral_limit = 10.0
        self.enabled = False

    def reset(self) -> None:
        """Clear accumulated integral and disable control."""
        self.integral = [0.0] * len(self.integral)
        self.enabled = False

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable control, clearing integral when re-enabled."""
        if enabled and not self.enabled:
            self.integral = [0.0] * len(self.integral)
        self.enabled = enabled

    def set_gains(
        self,
        kp: list[float] | None = None,
        ki: list[float] | None = None,
        kd: list[float] | None = None,
    ) -> None:
        """Apply independently optional gain arrays after validating them."""
        count = len(self.kp)
        if kp is not None:
            self.kp = validate_vector(kp, count, "kp")
        if ki is not None:
            self.ki = validate_vector(ki, count, "ki")
        if kd is not None:
            self.kd = validate_vector(kd, count, "kd")

    def effort(
        self,
        position: list[float],
        velocity: list[float],
        setpoint_position: list[float],
        setpoint_velocity: list[float],
        dt: float,
    ) -> list[float]:
        """Compute PID effort using commanded velocity for the derivative term."""
        if not self.enabled:
            return [0.0] * len(self.kp)
        output = []
        for i in range(len(self.kp)):
            error = setpoint_position[i] - position[i]
            self.integral[i] = max(-self.integral_limit, min(
                self.integral_limit, self.integral[i] + error * dt))
            velocity_error = setpoint_velocity[i] - velocity[i]
            output.append(self.kp[i] * error + self.ki[i] * self.integral[i]
                          + self.kd[i] * velocity_error)
        return output
