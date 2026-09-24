"""Timed IK trial harness node."""

import math
import random

from kinematics import forward_orientation, forward_position


class IKTrialNode:
    def __init__(self, action_node: object, publish=None) -> None:
        self.action_node = action_node
        self.publish = publish or (lambda _topic, _message: None)
        self.running = False
        self.duration = 30.0
        self.epsilon = 0.03
        self.success_hold = 0.2
        self.started = 0.0
        self.targets_reached = 0
        self.current_goal_id = None
        self.target = None
        self.action_status = "idle"
        self._seen_result = None

    def service_handlers(self) -> dict[str, object]:
        return {
            "/ik_trial/start": self.start,
            "/ik_trial/skip": self.skip,
            "/ik_trial/stop": self.stop,
        }

    def start(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        try:
            duration = _positive(args.get("duration", self.duration), "duration")
            epsilon = _positive(args.get("epsilon", self.epsilon), "epsilon")
            hold = _nonnegative(args.get("success_hold", self.success_hold), "success_hold")
        except ValueError as error:
            return False, {}, str(error)
        if self.action_node.active_goal is not None:
            self.action_node.cancel_goal({})
        self.duration, self.epsilon, self.success_hold = duration, epsilon, hold
        self.started = self.action_node.arm_sim.state.time
        self.targets_reached, self.running, self._seen_result = 0, True, None
        if not self._submit_target():
            self.running = False
            self._clear_goal()
            return False, {}, "could not sample a reachable target"
        return True, {}, ""

    def skip(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        if not self.running:
            return False, {}, "no trial is running"
        if self.action_node.active_goal is not None:
            self.action_node.cancel_goal({})
        if not self._submit_target():
            self.running = False
            self._clear_goal()
            return False, {}, "could not sample a reachable target"
        return True, {}, ""

    def stop(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        if not self.running:
            return False, {}, "no trial is running"
        if self.action_node.active_goal is not None:
            self.action_node.cancel_goal({})
        self.running = False
        self._clear_goal()
        return True, {}, ""

    def update(self, simulation_time: float) -> None:
        if self.running:
            result = self.action_node.last_result
            if result is not None and result["goal_id"] != self._seen_result:
                self._seen_result = result["goal_id"]
                self.action_status = result["outcome"]
                if result["goal_id"] == self.current_goal_id and result["outcome"] == "reached":
                    self.targets_reached += 1
                    self._submit_target()
            if simulation_time - self.started >= self.duration:
                if self.action_node.active_goal is not None:
                    self.action_node.cancel_goal({})
                self.running = False
                self._clear_goal()
        elapsed = max(0.0, simulation_time - self.started) if self.started else 0.0
        goal = self.action_node.active_goal
        error = self.action_node._distance(goal) if goal is not None and self.running else None
        self.publish("/ik_trial/status", {
            "running": self.running, "elapsed": elapsed, "duration": self.duration,
            "targets_reached": self.targets_reached, "target": self.target,
            "error": error, "desired_positions": None if goal is None or not self.running else goal["positions"][:],
            "action_status": self.action_status if self.running else "idle",
        })

    def _submit_target(self) -> bool:
        lengths = self.action_node.arm_sim.parameters.lengths
        for _ in range(100):
            # Sample joint angles then forward-project them.  Unlike a box or
            # disk sampler this remains reachable even after unusual runtime
            # length changes (including strongly unequal links).
            joints = [random.uniform(-math.pi, math.pi) for _ in lengths]
            x, y = forward_position(joints, lengths)
            request = {"x": x, "y": y, "epsilon": self.epsilon,
                       "success_hold": self.success_hold}
            if len(lengths) == 3:
                request["phi"] = forward_orientation(joints)
            success, values, _status = self.action_node.send_goal(request)
            if success:
                self.current_goal_id = values["goal_id"]
                self.target = {key: request[key] for key in ("x", "y", "phi") if key in request}
                self.action_status = "active"
                return True
        return False

    def _clear_goal(self) -> None:
        self.current_goal_id = None
        self.target = None
        self.action_status = "idle"


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _positive(value: object, name: str) -> float:
    value = _number(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _nonnegative(value: object, name: str) -> float:
    value = _number(value, name)
    if value < 0:
        raise ValueError(f"{name} must be non-negative")
    return value
