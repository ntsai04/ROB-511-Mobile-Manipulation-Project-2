"""Goal-oriented IK action node."""

import math
import uuid

from kinematics import forward_orientation, forward_position


class IKActionNode:
    def __init__(self, ik_node: object, arm_sim: object, publish=None) -> None:
        self.ik_node = ik_node
        self.arm_sim = arm_sim
        self.publish = publish or (lambda _topic, _message: None)
        self.active_goal = None
        self.last_result = None

    def service_handlers(self) -> dict[str, object]:
        return {
            "/ik_action/send_goal": self.send_goal,
            "/ik_action/cancel_goal": self.cancel_goal,
        }

    def send_goal(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        try:
            epsilon = _number(args.get("epsilon", 0.03), "epsilon")
            hold = _number(args.get("success_hold", 0.2), "success_hold")
            if epsilon <= 0 or hold < 0:
                raise ValueError("epsilon must be positive and success_hold must be non-negative")
        except ValueError as error:
            return False, {}, str(error)
        # Solving happens before changing active_goal: an unreachable request
        # cannot preempt a valid goal that is already in flight.
        success, values, status = self.ik_node.solve(args)
        if not success:
            return False, {}, status
        target = {"x": args["x"], "y": args["y"]}
        if self.arm_sim.links == 3 and "phi" in args:
            target["phi"] = args["phi"]
        if self.active_goal is not None:
            self._finish("preempted", self._distance(self.active_goal))
        goal_id = uuid.uuid4().hex
        self.active_goal = {"goal_id": goal_id, "target": target,
                            "positions": values["positions"], "epsilon": epsilon,
                            "success_hold": hold, "started": self.arm_sim.state.time,
                            "inside_since": None}
        self.publish("/joint_trajectory", {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
            "joint_names": [f"joint{i + 1}" for i in range(self.arm_sim.links)],
            "points": [{"positions": values["positions"], "velocities": [0.0] * self.arm_sim.links,
                        "accelerations": [], "time_from_start": {"sec": 0, "nanosec": 0}}],
        })
        # Internal publication must also update the servo, since local topics
        # are intentionally not required to advertise like TCP clients are.
        self.arm_sim.accept_trajectory({"points": [{"positions": values["positions"], "velocities": [0.0] * self.arm_sim.links}]})
        return True, {"goal_id": goal_id}, ""

    def cancel_goal(self, args: object) -> tuple[bool, dict, str]:
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        goal = self.active_goal
        requested = args.get("goal_id")
        if requested is not None and not isinstance(requested, str):
            return False, {}, "goal_id must be a string"
        if goal is None or (requested is not None and requested != goal["goal_id"]):
            return False, {}, "no matching active goal"
        self._finish("preempted", self._distance(goal))
        return True, {}, ""

    def update(self, simulation_time: float) -> None:
        goal = self.active_goal
        if goal is None:
            return
        distance = self._distance(goal)
        self.publish("/ik_action/feedback", {
            "goal_id": goal["goal_id"], "target": goal["target"], "positions": goal["positions"][:],
            "distance_remaining": distance, "elapsed": max(0.0, simulation_time - goal["started"]),
        })
        if distance <= goal["epsilon"]:
            if goal["inside_since"] is None:
                goal["inside_since"] = simulation_time
            if simulation_time - goal["inside_since"] >= goal["success_hold"]:
                self._finish("reached", distance)
        else:
            goal["inside_since"] = None

    def _distance(self, goal: dict) -> float:
        x, y = forward_position(self.arm_sim.state.position, self.arm_sim.parameters.lengths)
        return math.hypot(x - goal["target"]["x"], y - goal["target"]["y"])

    def _finish(self, outcome: str, distance: float) -> None:
        goal = self.active_goal
        if goal is None:
            return
        result = {"goal_id": goal["goal_id"], "outcome": outcome,
                  "target": goal["target"], "final_distance": distance}
        self.publish("/ik_action/result", result)
        self.last_result = result
        self.active_goal = None


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)
