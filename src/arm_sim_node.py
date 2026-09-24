"""Arm simulation node: dynamics, integration, PID, and simulation services."""

import os
import math
import threading

from arm_lib.arm_dynamics import ArmDynamics
from arm_lib.expr import parse_expression
from arm_lib.integrators import INTEGRATORS
from arm_lib.internal import ArmParameters, ArmState, Setpoint, validate_vector
from arm_lib.pid import PIDController


class ArmSimNode:
    def __init__(self, links: int | None = None) -> None:
        self.links = links or int(os.environ.get("ARM_SIM_LINKS", "2"))
        self.parameters = ArmParameters(self.links)
        self.state = ArmState([0.0] * self.links, [0.0] * self.links)
        self.setpoint = Setpoint([0.0] * self.links, [0.0] * self.links)
        self.dynamics = ArmDynamics(self.parameters)
        self.controller = PIDController(self.links)
        # Stable, useful defaults mean enabling the controller immediately is
        # demonstrable; callers remain free to replace every gain at runtime.
        self.controller.set_gains(kp=[40.0] * self.links, ki=[10.0] * self.links,
                                  kd=[25.0] * self.links)
        self.paused = False
        self.integrator = "rk4"
        self.timestep = 0.005
        self.last_effort = [0.0] * self.links
        self._lock = threading.RLock()

    def service_handlers(self) -> dict[str, object]:
        return {
            "/arm_sim/integration_step": self.integration_step,
            "/arm_sim/set_integrator": self.set_integrator,
            "/arm_sim/set_params": self.set_params,
            "/arm_sim/pause": self.pause,
            "/arm_sim/reset": self.reset,
            "/pid_controller/enable": self.enable_pid,
            "/pid_controller/set_gains": self.set_gains,
        }

    def integration_step(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        try:
            expression = parse_expression(args.get("function"))
            x0 = _number(args.get("x0"), "x0")
            xdot0 = _number(args.get("xdot0", 0.0), "xdot0")
            dt = _number(args.get("dt"), "dt")
            steps = args.get("steps")
            method = args.get("integrator")
            if dt <= 0:
                raise ValueError("dt must be positive")
            if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
                raise ValueError("steps must be a positive integer")
            if steps > 1_000_000:
                raise ValueError("steps is too large")
            if method not in INTEGRATORS:
                raise ValueError("unknown integrator")
        except (ValueError, TypeError) as error:
            return False, {}, str(error)
        q, qdot, time = [x0], [xdot0], 0.0
        times, positions, velocities = [time], [x0], [xdot0]
        def acceleration(now: float, _q: list[float], _v: list[float]) -> list[float]:
            return [expression(now)]
        for _ in range(steps):
            q, qdot = INTEGRATORS[method](acceleration, time, q, qdot, dt)
            time += dt
            times.append(time)
            positions.append(q[0])
            velocities.append(qdot[0])
        return True, {"times": times, "positions": positions, "velocities": velocities}, ""

    def set_integrator(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        method = args.get("method")
        try:
            timestep = _number(args.get("timestep"), "timestep")
        except ValueError as error:
            return False, self._integrator_values(), str(error)
        if method not in INTEGRATORS or timestep <= 0:
            return False, self._integrator_values(), "method must be a known integrator and timestep must be positive"
        with self._lock:
            self.integrator, self.timestep = method, timestep
            return True, self._integrator_values(), ""

    def set_params(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, self._parameter_values(), "arguments must be an object"
        errors = []
        with self._lock:
            if "gravity" in args:
                try:
                    gravity = _number(args["gravity"], "gravity")
                    if gravity < 0:
                        raise ValueError("gravity must be non-negative")
                    self.parameters.gravity = gravity
                except ValueError as error:
                    errors.append(str(error))
            for name in ("masses", "lengths"):
                if name not in args:
                    continue
                try:
                    vector = validate_vector(args[name], self.links, name)
                    if any(value <= 0 for value in vector):
                        raise ValueError(f"{name} entries must be positive")
                    setattr(self.parameters, name, vector)
                except ValueError as error:
                    errors.append(str(error))
            return not errors, self._parameter_values(), "; ".join(errors)

    def pause(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict) or not isinstance(args.get("data"), bool):
            return False, {"paused": self.paused}, "data must be a boolean"
        with self._lock:
            self.paused = args["data"]
            return True, {"paused": self.paused}, ""

    def reset(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        with self._lock:
            # Horizontal links give the disabled arm a non-equilibrium start.
            self.state = ArmState([0.0] * self.links, [0.0] * self.links)
            self.setpoint = Setpoint([0.0] * self.links, [0.0] * self.links)
            self.controller.reset()
            self.last_effort = [0.0] * self.links
            return True, {"position": self.state.position[:], "velocity": self.state.velocity[:]}, ""

    def enable_pid(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict) or not isinstance(args.get("data"), bool):
            return False, {"enabled": self.controller.enabled}, "data must be a boolean"
        with self._lock:
            self.controller.set_enabled(args["data"])
            return True, {"enabled": self.controller.enabled}, ""

    def set_gains(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, self._gain_values(), "arguments must be an object"
        errors = []
        with self._lock:
            for name in ("kp", "ki", "kd"):
                if name not in args:
                    continue
                try:
                    values = validate_vector(args[name], self.links, name)
                    if any(value < 0 for value in values):
                        raise ValueError(f"{name} entries must be non-negative")
                    self.controller.set_gains(**{name: values})
                except ValueError as error:
                    errors.append(str(error))
            return not errors, self._gain_values(), "; ".join(errors)

    def step(self, dt: float) -> None:
        with self._lock:
            if self.paused:
                return
            effort = self.controller.effort(
                self.state.position, self.state.velocity,
                self.setpoint.position, self.setpoint.velocity, dt,
            )
            self.last_effort = effort
            def acceleration(_time: float, position: list[float], velocity: list[float]) -> list[float]:
                return self.dynamics.acceleration(position, velocity, effort)
            position, velocity = INTEGRATORS[self.integrator](
                acceleration, self.state.time, self.state.position, self.state.velocity, dt)
            self.state.position, self.state.velocity = position, velocity
            self.state.time += dt

    def accept_trajectory(self, message: object) -> None:
        """Use only the final point as the documented held servo setpoint."""
        if not isinstance(message, dict) or not isinstance(message.get("points"), list) or not message["points"]:
            return
        point = message["points"][-1]
        if not isinstance(point, dict):
            return
        try:
            position = validate_vector(point.get("positions"), self.links, "positions")
        except ValueError:
            return
        try:
            velocity = validate_vector(point.get("velocities"), self.links, "velocities")
        except ValueError:
            velocity = [0.0] * self.links
        with self._lock:
            self.setpoint = Setpoint(position, velocity)

    def joint_state_message(self) -> dict:
        with self._lock:
            seconds = int(self.state.time)
            nanoseconds = int((self.state.time - seconds) * 1_000_000_000)
            return {"header": {"stamp": {"sec": seconds, "nanosec": nanoseconds}, "frame_id": ""},
                    "name": [f"joint{i + 1}" for i in range(self.links)],
                    "position": self.state.position[:], "velocity": self.state.velocity[:],
                    "effort": self.last_effort[:]}

    def _parameter_values(self) -> dict:
        return {"gravity": self.parameters.gravity, "masses": self.parameters.masses[:], "lengths": self.parameters.lengths[:]}

    def _gain_values(self) -> dict:
        return {"kp": self.controller.kp[:], "ki": self.controller.ki[:], "kd": self.controller.kd[:]}

    def _integrator_values(self) -> dict:
        return {"method": self.integrator, "timestep": self.timestep}


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)
