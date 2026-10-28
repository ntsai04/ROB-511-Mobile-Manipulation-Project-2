"""Offline Pendularm algorithm visualizer and optional live-runtime bridge."""

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))
from rosbridge_client import Rosbridge


def arm_points(q, lengths):
    from kinematics import forward_position
    return [[0, 0]] + [list(forward_position(q[:i+1], lengths[:i+1])) for i in range(len(q))]


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT / "visualizer"), **kwargs)

    def log_message(self, *_args):
        pass

    def reply(self, body, status=200):
        data = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path != "/api/state":
            return super().do_GET()
        try:
            with Rosbridge(port=self.server.runtime_port, timeout=2) as ros:
                ros.subscribe("/joint_states", "sensor_msgs/JointState", timeout=2)
                state = ros.next_message("/joint_states", timeout=2)
                params = ros.call_service("/arm_sim/set_params", {}, timeout=2)
            self.reply({"state": state, "parameters": params["values"],
                        "points": arm_points(state["position"], params["values"]["lengths"])})
        except Exception as error:
            self.reply({"error": str(error), "hint": "Start make run in a second terminal for live state."}, 503)

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 65536:
                raise ValueError("request size must be between 1 and 65536 bytes")
            request = json.loads(self.rfile.read(length))
            result = self.evaluate(request)
            self.reply(result)
        except Exception as error:
            self.reply({"error": str(error)}, 400)

    def evaluate(self, request):
        if self.path == "/api/ik":
            from kinematics import solve_ik, forward_position, forward_orientation
            lengths = request["lengths"]
            if len(lengths) not in (2, 3) or any(x <= 0 for x in lengths):
                raise ValueError("Use two or three positive link lengths.")
            target = request["target"]
            q = solve_ik(target[0], target[1], lengths, request.get("phi", 0))
            return {"joints": q, "position": forward_position(q, lengths),
                    "points": arm_points(q, lengths), "orientation": forward_orientation(q)}
        if self.path == "/api/integrate":
            from arm_sim_node import ArmSimNode
            if request.get("steps", 0) > 2000:
                raise ValueError("Visualizer allows at most 2000 steps.")
            ok, data, status = ArmSimNode(2).integration_step(request)
            if not ok:
                raise ValueError(status)
            return data
        if self.path == "/api/pid":
            from arm_lib.pid import PIDController
            pid = PIDController(1)
            kp, ki, kd = request["gains"]
            pid.set_gains(kp=[kp], ki=[ki], kd=[kd])
            pid.set_enabled(True)
            position, velocity, target, target_velocity, dt = request["state"]
            effort = pid.effort([position], [velocity], [target], [target_velocity], dt)[0]
            return {"effort": effort, "integral": pid.integral[0]}
        if self.path == "/api/dynamics":
            from arm_lib.arm_dynamics import ArmDynamics
            from arm_lib.internal import ArmParameters
            q, v, torque = request["q"], request["velocity"], request["torque"]
            lengths, masses, gravity = request["lengths"], request["masses"], request["gravity"]
            if len(q) not in (2, 3) or any(len(x) != len(q) for x in (v, torque, lengths, masses)):
                raise ValueError("Use matching arrays of two or three values.")
            dynamics = ArmDynamics(ArmParameters(len(q), gravity, masses, lengths))
            matrix = dynamics.mass_matrix(q)
            coriolis, load = dynamics.coriolis_load(q, v), dynamics.gravity_load(q)
            acceleration = dynamics.acceleration(q, v, torque)
            return {"mass": matrix, "coriolis": coriolis, "gravity": load,
                    "acceleration": acceleration, "points": arm_points(q, lengths)}
        if self.path == "/api/live":
            allowed = {"/arm_sim/pause", "/arm_sim/reset", "/pid_controller/enable", "/ik_action/send_goal",
                       "/arm_sim/set_integrator", "/pid_controller/set_gains"}
            if request.get("service") not in allowed:
                raise ValueError("Unsupported live control.")
            with Rosbridge(port=self.server.runtime_port, timeout=2) as ros:
                return ros.call_service(request["service"], request.get("args", {}), timeout=3)
        raise ValueError("Unknown endpoint")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8002)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--runtime-port", type=int, default=9095)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.runtime_port = args.runtime_port
    print(f"Pendularm visualizer: http://localhost:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
