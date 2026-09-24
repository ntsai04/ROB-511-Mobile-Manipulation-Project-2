"""Thread-safe registry for rosbridge topics and services.

The hub has no socket knowledge.  It records subscriptions, service providers,
and in-flight forwarded calls so the gateway can route messages without
holding a lock while writing to a client.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


# These declarations keep the Project 2 node graph in one place.  They are
# descriptive only; nodes may be implemented in one process or several.
PROJECT_NODES = {
    "arm_sim": {
        "services": (
            "/arm_sim/integration_step",
            "/arm_sim/set_integrator",
            "/arm_sim/set_params",
            "/arm_sim/pause",
            "/arm_sim/reset",
            "/pid_controller/enable",
            "/pid_controller/set_gains",
        ),
        "publishes": ("/joint_states",),
        "subscribes": ("/joint_trajectory",),
    },
    "ik": {
        "services": ("/ik/solve",),
        "publishes": (),
        "subscribes": (),
    },
    "ik_action": {
        "services": ("/ik_action/send_goal", "/ik_action/cancel_goal"),
        "publishes": ("/ik_action/feedback", "/ik_action/result"),
        "subscribes": (),
    },
    "ik_trial": {
        "services": ("/ik_trial/start", "/ik_trial/skip", "/ik_trial/stop"),
        "publishes": ("/ik_trial/status",),
        "subscribes": (),
    },
}


@dataclass
class PendingCall:
    caller: Any
    caller_id: Any
    service: str
    deadline: float


class Hub:
    """Registry shared by all connected clients."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subscribers: dict[str, set[Any]] = {}
        self._providers: dict[str, Any] = {}
        self._pending: dict[tuple[Any, str], PendingCall] = {}

    def subscribe(self, client: Any, topic: str) -> None:
        with self._lock:
            self._subscribers.setdefault(topic, set()).add(client)
            client.subscriptions.add(topic)

    def unsubscribe(self, client: Any, topic: str) -> None:
        with self._lock:
            subscribers = self._subscribers.get(topic)
            if subscribers is not None:
                subscribers.discard(client)
                if not subscribers:
                    self._subscribers.pop(topic, None)
            client.subscriptions.discard(topic)

    def publication_recipients(self, topic: str) -> list[Any]:
        with self._lock:
            return list(self._subscribers.get(topic, ()))

    def advertise_service(self, client: Any, service: str) -> None:
        with self._lock:
            previous = self._providers.get(service)
            if previous is not None:
                previous.services.discard(service)
            self._providers[service] = client
            client.services.add(service)

    def unadvertise_service(self, client: Any, service: str) -> None:
        with self._lock:
            if self._providers.get(service) is client:
                self._providers.pop(service, None)
            client.services.discard(service)

    def provider_for(self, service: str) -> Any | None:
        with self._lock:
            return self._providers.get(service)

    def add_pending_call(self, provider: Any, provider_id: str, call: PendingCall) -> None:
        with self._lock:
            self._pending[(provider, provider_id)] = call

    def take_pending_call(self, provider: Any, provider_id: str) -> PendingCall | None:
        with self._lock:
            return self._pending.pop((provider, provider_id), None)

    def expired_calls(self, now: float | None = None) -> list[PendingCall]:
        now = time.monotonic() if now is None else now
        with self._lock:
            expired_keys = [
                key for key, call in self._pending.items()
                if call.deadline <= now
            ]
            expired = [self._pending.pop(key) for key in expired_keys]
            return expired

    def disconnect(self, client: Any) -> list[PendingCall]:
        with self._lock:
            for topic in tuple(client.subscriptions):
                self.unsubscribe(client, topic)

            for service in tuple(client.services):
                if self._providers.get(service) is client:
                    self._providers.pop(service, None)
            client.services.clear()

            failed = []
            for key, call in list(self._pending.items()):
                provider, _ = key
                caller_left = call.caller is client
                provider_left = provider is client
                if caller_left or provider_left:
                    self._pending.pop(key)
                    if provider_left and not caller_left:
                        failed.append(call)
            return failed
