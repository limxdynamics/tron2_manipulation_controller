from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .protocol import build_request, expected_response_title
from .state import JointState, MovePose, StateCache


class ProtocolError(RuntimeError):
    pass


class DualArmProtocolClient:
    def __init__(
        self,
        robot_ip: str,
        *,
        port: int = 5000,
        accid: Optional[str] = None,
        response_timeout_sec: float = 5.0,
        monotonic: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.robot_ip = robot_ip
        self.port = port
        self.accid = accid
        self.response_timeout_sec = response_timeout_sec
        self._monotonic = monotonic
        self.state_cache = StateCache()
        self._ws = None
        self._recv_thread: Optional[threading.Thread] = None
        self._closed = threading.Event()
        self._condition = threading.Condition()
        self._responses: Dict[str, Dict[str, Any]] = {}
        self._response_received_at: Dict[str, float] = {}
        self._notifications: List[Dict[str, Any]] = []

    @property
    def url(self) -> str:
        host = self.robot_ip.strip()
        if host.startswith("[") and host.endswith("]"):
            host = host[1:-1]
        if ":" in host:
            if "%" in host:
                address, zone = host.split("%", 1)
                if not zone.startswith("25"):
                    host = f"{address}%25{zone}"
            host = f"[{host}]"
        return f"ws://{host}:{self.port}"

    def connect(self, timeout_sec: float = 5.0) -> None:
        try:
            import websocket
        except ImportError as exc:
            raise ProtocolError(
                "Missing dependency 'websocket-client'. Install requirements.txt first."
            ) from exc

        self._ws = websocket.create_connection(self.url, timeout=timeout_sec)
        self._closed.clear()
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def close(self) -> None:
        self._closed.set()
        if self._ws is not None:
            self._ws.close()
            self._ws = None
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)

    def request(
        self,
        title: str,
        data: Optional[Dict[str, Any]] = None,
        *,
        expect_response: bool = True,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        message = build_request(title, data, accid=self.accid)
        if self._ws is None:
            raise ProtocolError("WebSocket is not connected")
        self._ws.send(json.dumps(message, ensure_ascii=False))
        if not expect_response:
            return message
        return self.wait_response(
            message["guid"],
            expected_response_title(title),
            timeout_sec=timeout_sec or self.response_timeout_sec,
        )

    def send_command(
        self,
        name: str,
        title: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        message = build_request(title, data, accid=self.accid)
        if self._ws is None:
            raise ProtocolError("WebSocket is not connected")
        sent_at = self._monotonic()
        self._ws.send(json.dumps(message, ensure_ascii=False))
        return {
            "name": name,
            "guid": message["guid"],
            "response_title": expected_response_title(title),
            "sent_at": sent_at,
        }

    def wait_commands(
        self,
        commands: List[Dict[str, Any]],
        *,
        timeout_sec: float,
    ) -> List[Dict[str, Any]]:
        deadline = self._monotonic() + timeout_sec
        results: Dict[str, Dict[str, Any]] = {}
        pending = {command["guid"]: command for command in commands}
        with self._condition:
            while pending and self._monotonic() < deadline:
                for guid, command in list(pending.items()):
                    response = self._responses.get(guid)
                    if response is None:
                        continue
                    self._responses.pop(guid, None)
                    received_at = self._response_received_at.pop(
                        guid,
                        None,
                    )
                    if received_at is None:
                        received_at = self._monotonic()
                    pending.pop(guid, None)
                    status = str((response.get("data") or {}).get("result", ""))
                    title_ok = response.get("title") == command["response_title"]
                    results[guid] = {
                        "name": command["name"],
                        "ok": title_ok and status == "success",
                        "status": status or "empty_result",
                        "elapsed_ms": (
                            received_at - command["sent_at"]
                        )
                        * 1000.0,
                        "response": response,
                    }
                if pending:
                    remaining = max(
                        0.0,
                        deadline - self._monotonic(),
                    )
                    self._condition.wait(timeout=min(0.05, remaining))

            for guid, command in pending.items():
                self._responses.pop(guid, None)
                self._response_received_at.pop(guid, None)
                results[guid] = {
                    "name": command["name"],
                    "ok": False,
                    "status": "timeout",
                    "elapsed_ms": (
                        self._monotonic() - command["sent_at"]
                    )
                    * 1000.0,
                    "response": None,
                }
        return [results[command["guid"]] for command in commands]

    def wait_response(self, guid: str, title: str, *, timeout_sec: float) -> Dict[str, Any]:
        deadline = self._monotonic() + timeout_sec
        with self._condition:
            try:
                while self._monotonic() < deadline:
                    response = self._responses.get(guid)
                    if response and response.get("title") == title:
                        return self._responses.pop(guid)
                    remaining = max(
                        0.0,
                        deadline - self._monotonic(),
                    )
                    self._condition.wait(timeout=min(0.1, remaining))
            finally:
                self._responses.pop(guid, None)
                self._response_received_at.pop(guid, None)
        raise TimeoutError(f"Timed out waiting for {title} guid={guid}")

    def drain_notifications(self, title: Optional[str] = None) -> List[Dict[str, Any]]:
        with self._condition:
            if title is None:
                items = list(self._notifications)
                self._notifications.clear()
                return items
            matched = [item for item in self._notifications if item.get("title") == title]
            self._notifications = [
                item for item in self._notifications if item.get("title") != title
            ]
            return matched

    def get_joint_state(self) -> JointState:
        state = JointState.from_response(self.request("request_get_joint_state", {}))
        return self.state_cache.update_joint_state(state)

    def get_move_pose(self) -> MovePose:
        pose = MovePose.from_response(self.request("request_get_move_pose", {}))
        return self.state_cache.update_move_pose(pose)

    def wait_for_accid(self, timeout_sec: float = 5.0) -> Optional[str]:
        deadline = self._monotonic() + timeout_sec
        with self._condition:
            while self.accid is None and self._monotonic() < deadline:
                self._condition.wait(timeout=0.1)
        return self.accid

    def _recv_loop(self) -> None:
        while not self._closed.is_set() and self._ws is not None:
            try:
                raw = self._ws.recv()
            except Exception:
                break
            if not raw:
                continue
            try:
                message = json.loads(raw)
            except json.JSONDecodeError:
                continue
            self._handle_message(message)

    def _handle_message(self, message: Dict[str, Any]) -> None:
        with self._condition:
            if self.accid is None and message.get("accid"):
                self.accid = message["accid"]
            title = str(message.get("title", ""))
            if title.startswith("response_") and message.get("guid"):
                guid = message["guid"]
                self._responses[guid] = message
                self._response_received_at[guid] = self._monotonic()
            elif title.startswith("notify_"):
                self._notifications.append(message)
            self._condition.notify_all()
