"""TCP/JSON rosbridge gateway on 127.0.0.1:9095.

External clients send one JSON object per line.  A reader thread handles each
connection and a writer queue serializes outbound messages.
"""
from __future__ import annotations

import json
import queue
import socket
import sys
import threading
import time
import uuid
from typing import Any

try:
    from src.hub import Hub, PendingCall
except ModuleNotFoundError:
    from hub import Hub, PendingCall


HOST = "127.0.0.1"
PORT = 9095
SERVICE_TIMEOUT_SECONDS = 5.0


class ClientConnection:
    """One external TCP client: socket, outbound queue, and Hub registrations."""

    def __init__(self, server: "RosbridgeServer", connection: socket.socket) -> None:
        self.server = server
        self.connection = connection
        # Writer waits on this queue.  dict = JSON to send.  None = shut down.
        self.outgoing: queue.Queue[dict[str, Any] | None] = queue.Queue()
        self.subscriptions: set[str] = set()
        self.publishers: set[str] = set()
        self.services: set[str] = set()
        self._closed = False
        self._close_lock = threading.Lock()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)

    def start(self) -> None:
        """Start the writer thread.  The reader is the thread that called us."""
        self._writer.start()

    def send(self, message: dict[str, Any]) -> None:
        """Enqueue one outbound JSON object.  Ignored if close() already ran."""
        with self._close_lock:
            if not self._closed:
                self.outgoing.put(message)

    def _write_loop(self) -> None:
        """Take queued messages and write them as UTF-8 JSON lines.

        OSError covers a peer that already closed.  The finally block always
        unregisters this client so Hub does not keep a dead socket.
        """
        try:
            while True:
                message = self.outgoing.get()
                if message is None:
                    return
                encoded = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
                self.connection.sendall(encoded + b"\n")
        except OSError:
            pass
        finally:
            self.server.disconnect_client(self)

    def close(self) -> None:
        """Idempotent shutdown: one stop-sentinel, then close the socket.

        _closed is set first so late send() calls do not queue work after we
        have already asked the writer to exit.  shutdown() unblocks a reader
        stuck in readline.  close() is wrapped because the peer may have gone.
        """
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self.outgoing.put(None)
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.connection.close()
            except OSError:
                pass


class RosbridgeServer:
    """Shared state for every client: Hub registry plus the A* planner node."""

    def __init__(self) -> None:
        self.hub = Hub()
        self.planner = None
        self.local_services = {}
        self.topic_listeners: dict[str, list[Any]] = {}
        try:
            from src.heap_services import handle_heap_service
        except ImportError:
            try:
                from heap_services import handle_heap_service
            except ImportError:
                handle_heap_service = None
        if handle_heap_service is not None:
            self.local_services["/heapify"] = handle_heap_service
            self.local_services["/heap_sort"] = handle_heap_service
        try:
            from src.astar_node import AStarNode
        except ImportError:
            try:
                from astar_node import AStarNode
            except ImportError:
                AStarNode = None
        if AStarNode is not None:
            self.planner = AStarNode()

    def register_local_service(self, name: str, handler: Any) -> None:
        """Register an in-process service without adding gateway dispatch code."""
        self.local_services[name] = handler

    def register_topic_listener(self, topic: str, listener: Any) -> None:
        """Register an in-process subscriber used by the simulation nodes."""
        self.topic_listeners.setdefault(topic, []).append(listener)

    def publish_local(self, topic: str, message: Any) -> None:
        """Publish from an in-process node without a TCP advertisement."""
        for listener in self.topic_listeners.get(topic, ()):
            listener(message)
        self._broadcast(topic, message)

    @staticmethod
    def send_status(client: ClientConnection, level: str, message: str,
                    request: dict[str, Any]) -> None:
        """rosbridge status message.  Echo request id when the client sent one."""
        envelope: dict[str, Any] = {"op": "status", "level": level, "msg": message}
        if "id" in request:
            envelope["id"] = request["id"]
        client.send(envelope)

    @staticmethod
    def send_service_response(client: ClientConnection, service: str, request_id: Any,
                              result: bool, values: Any = None, status: str = "") -> None:
        """Standard service_response.  Missing values become an empty object."""
        client.send({
            "op": "service_response",
            "service": service,
            "id": request_id,
            "values": {} if values is None else values,
            "result": result,
            "status": status,
        })

    def disconnect_client(self, client: ClientConnection) -> None:
        """Unregister in Hub, close the socket, tell waiting callers the provider died.

        close() is safe to call twice (reader finally + writer finally).
        """
        failures = self.hub.disconnect(client)
        client.close()
        for pending in failures:
            self.send_service_response(
                pending.caller, pending.service, pending.caller_id,
                False, status="service provider disconnected",
            )

    def expire_calls_forever(self) -> None:
        """Background loop: fail forwarded calls that sat longer than the timeout."""
        while True:
            time.sleep(0.25)
            for pending in self.hub.expired_calls():
                self.send_service_response(
                    pending.caller, pending.service, pending.caller_id,
                    False, status="service call timed out",
                )

    def handle_request(self, client: ClientConnection, request: Any) -> None:
        """Dispatch one parsed JSON object by its ``op`` string."""
        if not isinstance(request, dict) or not isinstance(request.get("op"), str):
            self.send_status(client, "error", "request must contain a string op", {})
            return

        operation = request["op"]
        handlers = {
            "advertise": self._advertise,
            "unadvertise": self._unadvertise,
            "publish": self._publish,
            "subscribe": self._subscribe,
            "unsubscribe": self._unsubscribe,
            "advertise_service": self._advertise_service,
            "unadvertise_service": self._unadvertise_service,
            "call_service": self._call_service,
            "service_response": self._forward_service_response,
        }
        handler = handlers.get(operation)
        if handler is None:
            self.send_status(client, "error", f"unknown operation: {operation}", request)
            return
        handler(client, request)

    def _advertise(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Claim the right to publish on a topic.  Stored only on the client."""
        topic = request.get("topic")
        if isinstance(topic, str):
            client.publishers.add(topic)
        else:
            self.send_status(client, "error", "advertise requires topic", request)

    def _unadvertise(self, client: ClientConnection, request: dict[str, Any]) -> None:
        topic = request.get("topic")
        if isinstance(topic, str):
            client.publishers.discard(topic)
        else:
            self.send_status(client, "error", "unadvertise requires topic", request)

    def _publish(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Fan a message out to subscribers.  /map also updates the A* grid."""
        topic = request.get("topic")
        if not isinstance(topic, str) or "msg" not in request:
            self.send_status(client, "error", "publish requires topic and msg", request)
            return
        if topic not in client.publishers:
            self.send_status(client, "error", "publish requires advertised topic", request)
            return
        if topic == "/map" and self.planner is not None:
            self.planner.set_map(request["msg"])
        for listener in self.topic_listeners.get(topic, ()):
            listener(request["msg"])
        self._broadcast(topic, request["msg"])

    def _broadcast(self, topic: str, msg: Any) -> None:
        """Copy recipients from Hub, then send without holding Hub's lock."""
        envelope = {"op": "publish", "topic": topic, "msg": msg}
        for subscriber in self.hub.publication_recipients(topic):
            subscriber.send(envelope)

    def _subscribe(self, client: ClientConnection, request: dict[str, Any]) -> None:
        topic = request.get("topic")
        if not isinstance(topic, str):
            self.send_status(client, "error", "subscribe requires topic", request)
            return
        self.hub.subscribe(client, topic)
        self.send_status(client, "info", f"subscribed to {topic}", request)

    def _unsubscribe(self, client: ClientConnection, request: dict[str, Any]) -> None:
        topic = request.get("topic")
        if isinstance(topic, str):
            self.hub.unsubscribe(client, topic)
        else:
            self.send_status(client, "error", "unsubscribe requires topic", request)

    def _advertise_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Last advertiser becomes the unique provider of this service name."""
        service = request.get("service")
        if not isinstance(service, str):
            self.send_status(client, "error", "advertise_service requires service", request)
            return
        self.hub.advertise_service(client, service)
        self.send_status(client, "info", f"advertised service {service}", request)

    def _unadvertise_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        service = request.get("service")
        if isinstance(service, str):
            self.hub.unadvertise_service(client, service)
        else:
            self.send_status(client, "error", "unadvertise_service requires service", request)

    def _call_service(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Answer locally, or forward to the current provider.

        Required fields: service (string), id (any JSON), args (any JSON).
        Local heap/plan calls never create a PendingCall.  Forwarded calls do.
        """
        service = request.get("service")
        if not isinstance(service, str) or "id" not in request or "args" not in request:
            self.send_status(client, "error", "call_service requires service, id, and args", request)
            return

        # Heap services are implemented in this process.
        local_handler = self.local_services.get(service)
        if local_handler is not None:
            result, values, status = local_handler(service, request["args"])
            self.send_service_response(client, service, request["id"], result, values, status)
            return

        # A* is also local.  A successful plan is both the service result
        # and a publish on /path so anyone subscribed to /path sees it.
        if service == "/plan_path" and self.planner is not None:
            result, values, status = self.planner.plan_path(request["args"])
            if result:
                self._broadcast("/path", values["plan"])
            self.send_service_response(client, service, request["id"], result, values, status)
            return

        provider = self.hub.provider_for(service)
        if provider is None:
            self.send_service_response(
                client, service, request["id"], False, status="service unavailable",
            )
            return

        # Caller ids are only unique *per connection*.  Two clients can both
        # send id "call-1".  We invent a unique provider_id so the later
        # service_response cannot be matched to the wrong caller.
        provider_id = "provider-call-" + uuid.uuid4().hex
        deadline = time.monotonic() + SERVICE_TIMEOUT_SECONDS
        self.hub.add_pending_call(
            provider,
            provider_id,
            PendingCall(client, request["id"], service, deadline),
        )
        provider.send({
            "op": "call_service",
            "service": service,
            "id": provider_id,
            "args": request["args"],
        })

    def _forward_service_response(self, client: ClientConnection, request: dict[str, Any]) -> None:
        """Provider answered.  Look up the original caller and rewrite the id.

        Unknown or already-expired ids are ignored (timeout may have already
        sent result:false).  bool(...) turns missing/null result into False.
        """
        provider_id = request.get("id")
        if not isinstance(provider_id, str):
            return
        pending = self.hub.take_pending_call(client, provider_id)
        if pending is None:
            return
        status = request.get("status", "")
        if not isinstance(status, str):
            status = ""
        self.send_service_response(
            pending.caller,
            pending.service,
            pending.caller_id,
            bool(request.get("result")),
            request.get("values", {}),
            status,
        )


def serve_client(server: RosbridgeServer, connection: socket.socket) -> None:
    """Reader thread for one socket: one JSON object per line, then disconnect.

    makefile(..., newline="\\n") yields each line including the newline, which
    json.loads accepts.  Invalid JSON is a status error, not a disconnect.
    OSError/UnicodeError mean the peer left or sent bad bytes: just clean up.
    """
    client = ClientConnection(server, connection)
    client.start()
    try:
        with connection.makefile("r", encoding="utf-8", newline="\n") as incoming:
            for line in incoming:
                try:
                    server.handle_request(client, json.loads(line))
                except json.JSONDecodeError:
                    server.send_status(client, "error", "invalid JSON", {})
    except (OSError, UnicodeError):
        pass
    finally:
        server.disconnect_client(client)


def main() -> int:
    """Listen on HOST:PORT.  Each accept() gets its own daemon reader thread."""
    server = RosbridgeServer()
    threading.Thread(target=server.expire_calls_forever, daemon=True).start()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            # SO_REUSEADDR lets us bind again quickly after a restart.
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((HOST, PORT))
            listener.listen()
            print(f"Autorob gateway listening on {HOST}:{PORT}", file=sys.stderr, flush=True)
            while True:
                connection, _address = listener.accept()
                threading.Thread(
                    target=serve_client, args=(server, connection), daemon=True,
                ).start()
    except OSError as error:
        print(f"unable to start gateway: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
