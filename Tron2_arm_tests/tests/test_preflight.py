from __future__ import annotations

from typing import Any, Dict, List

from dual_arm_protocol_tests.preflight import run_preflight
from dual_arm_protocol_tests.robot_model import DEFAULT_HOME_Q16


class FakeClient:
    def __init__(
        self,
        response: Dict[str, Any],
        *,
        notifications: List[Dict[str, Any]] | None = None,
    ) -> None:
        self.response = response
        self.notifications = list(notifications or [])
        self.requests: List[Dict[str, Any]] = []

    def drain_notifications(self, title: str) -> List[Dict[str, Any]]:
        matched = [
            item for item in self.notifications if item.get("title") == title
        ]
        self.notifications = [
            item for item in self.notifications if item.get("title") != title
        ]
        return matched

    def request(
        self,
        title: str,
        data: Dict[str, Any],
        *,
        timeout_sec: float,
    ) -> Dict[str, Any]:
        self.requests.append(
            {"title": title, "data": data, "timeout_sec": timeout_sec}
        )
        return self.response


def _joint_state_response(*, result: str = "success", dq: List[float] | None = None) -> Dict[str, Any]:
    return {
        "data": {
            "result": result,
            "q": list(DEFAULT_HOME_Q16),
            "dq": list(dq if dq is not None else [0.0] * 16),
        }
    }


def test_run_preflight_passes_for_stable_joint_state() -> None:
    client = FakeClient(_joint_state_response())

    result = run_preflight(client, {"thresholds": {"stable_velocity_rad_s": 0.05}})

    assert result.ok is True
    assert result.code == "ok"
    assert result.max_abs_dq == 0.0
    assert client.requests[0]["title"] == "request_get_joint_state"


def test_run_preflight_rejects_developer_mode_failure() -> None:
    client = FakeClient(_joint_state_response(result="fail_is_not_develop_mode"))

    result = run_preflight(client, {})

    assert result.ok is False
    assert result.code == "developer_mode_required"


def test_run_preflight_rejects_dangerous_robot_status_before_request() -> None:
    client = FakeClient(
        _joint_state_response(),
        notifications=[
            {"title": "notify_robot_info", "data": {"status": "ERROR"}},
        ],
    )

    result = run_preflight(client, {})

    assert result.ok is False
    assert result.code == "dangerous_status"
    assert client.requests == []


def test_run_preflight_rejects_unstable_joint_velocity() -> None:
    dq = [0.0] * 16
    dq[3] = 0.10
    client = FakeClient(_joint_state_response(dq=dq))

    result = run_preflight(client, {"thresholds": {"stable_velocity_rad_s": 0.05}})

    assert result.ok is False
    assert result.code == "robot_not_stationary"
    assert result.max_abs_dq == 0.10


def test_run_preflight_rejects_short_joint_state() -> None:
    client = FakeClient({"data": {"result": "success", "q": [0.0], "dq": []}})

    result = run_preflight(client, {})

    assert result.ok is False
    assert result.code == "invalid_joint_state"
