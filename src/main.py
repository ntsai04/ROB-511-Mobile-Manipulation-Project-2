"""Pendularm runtime: compose local nodes with the rosbridge TCP gateway.

The public boundary is deliberately small: a TCP/JSON gateway owns sockets,
while the arm, IK, action, and trial nodes own their domain logic.  They run in
one process here, but communicate through the same service/topic shapes an
external node would use.
"""

from __future__ import annotations

import os
import threading
import time

from arm_sim_node import ArmSimNode
from ik_action_node import IKActionNode
from ik_node import IKNode
from ik_trial_node import IKTrialNode
from rosbridge_server import RosbridgeServer, serve_client, HOST, PORT
import socket
import sys


def _register_services(server: RosbridgeServer, node: object) -> None:
    """Expose every service supplied by one local node through the gateway."""
    for name, handler in node.service_handlers().items():
        # The gateway's existing local-service ABI includes the service name;
        # project nodes deliberately only need their JSON argument object.
        server.register_local_service(name, lambda _name, args, handler=handler: handler(args))


def main() -> int:
    raw_links = os.environ.get("ARM_SIM_LINKS", "2")
    if raw_links not in ("2", "3"):
        print("ARM_SIM_LINKS must be 2 or 3", file=sys.stderr)
        return 2
    arm = ArmSimNode(int(raw_links))
    server = RosbridgeServer()
    ik = IKNode(arm.parameters)
    action = IKActionNode(ik, arm, server.publish_local)
    trial = IKTrialNode(action, server.publish_local)
    _register_services(server, arm)
    server.register_local_service("/ik/solve", lambda _name, args: ik.solve(args))
    _register_services(server, action)
    _register_services(server, trial)
    server.register_topic_listener("/joint_trajectory", arm.accept_trajectory)

    def simulation_loop() -> None:
        """Advance physics and publish snapshots without blocking TCP clients."""
        last_publish = 0.0
        while True:
            # Each update uses the selected numerical timestep.  The short
            # sleep merely yields CPU time; it never changes the integration
            # method or the amount of simulated time in this step.
            arm.step(arm.timestep)
            action.update(arm.state.time)
            trial.update(arm.state.time)
            now = time.monotonic()
            if now - last_publish >= 1 / 60:
                server.publish_local("/joint_states", arm.joint_state_message())
                last_publish = now
            time.sleep(0.005)

    threading.Thread(target=simulation_loop, daemon=True).start()
    threading.Thread(target=server.expire_calls_forever, daemon=True).start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((HOST, PORT))
            listener.listen()
            print(f"Pendularm listening on {HOST}:{PORT}", file=sys.stderr, flush=True)
            while True:
                connection, _address = listener.accept()
                threading.Thread(target=serve_client, args=(server, connection), daemon=True).start()
    except OSError as error:
        print(f"unable to start gateway: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
