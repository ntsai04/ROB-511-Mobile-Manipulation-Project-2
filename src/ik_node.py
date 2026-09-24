"""Inverse-kinematics service node."""

import math

from arm_lib.internal import ArmParameters
from kinematics import solve_ik


class IKNode:
    def __init__(self, parameters: ArmParameters) -> None:
        self.parameters = parameters

    def solve(self, args: object) -> tuple[bool, dict, str]:
        """Handle /ik/solve and return a rosbridge service tuple."""
        if not isinstance(args, dict):
            return False, {}, "arguments must be an object"
        try:
            x, y = _number(args.get("x"), "x"), _number(args.get("y"), "y")
            phi = None if "phi" not in args else _number(args["phi"], "phi")
            # Parameters are shared with arm_sim, so this is always a fresh
            # read after any successful set_params update.
            positions = solve_ik(x, y, self.parameters.lengths[:], phi)
            return True, {"positions": positions}, ""
        except ValueError as error:
            return False, {}, str(error)


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)
