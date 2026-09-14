from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import math

from .state import JointState


DANGEROUS_STATUSES = {
    "ERROR",
    "ERROR_FALLOVER",
    "ERROR_RECOVER",
    "RECOVER",
    "ST_FALLOVER",
    "ST_RECOVERING",
    "ST_CALIBRATING",
}


@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    code: str
    message: str
    joint_state: Optional[JointState] = None
    robot_info: Dict[str, Any] = field(default_factory=dict)
    max_abs_dq: float = 0.0

    def as_row(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "code": self.code,
            "message": self.message,
            "robot_info": self.robot_info,
            "joint_q": self.joint_state.q if self.joint_state else [],
            "joint_dq": self.joint_state.dq if self.joint_state else [],
            "max_abs_dq": self.max_abs_dq,
        }


def run_preflight(client: Any, config: Dict[str, Any]) -> PreflightResult:
    robot_info = _latest_robot_info(client)
    info_data = robot_info.get("data", {}) if robot_info else {}
    status = str(info_data.get("status", "")).strip()
    if status in DANGEROUS_STATUSES:
        return PreflightResult(False, "dangerous_status", f"机器人处于危险状态: {status}", None, info_data)

    diagnostics = {key: str(info_data.get(key, "OK")).upper() for key in ("imu", "motor")}
    bad = [key for key, value in diagnostics.items() if value and value != "OK"]
    if bad:
        return PreflightResult(
            False,
            "diagnostics_not_ok",
            f"诊断状态异常: {', '.join(bad)}",
            None,
            info_data,
        )

    try:
        response = client.request("request_get_joint_state", {}, timeout_sec=_response_timeout(config))
    except Exception as exc:
        return PreflightResult(False, "joint_state_unavailable", str(exc), None, info_data)

    result = str((response.get("data") or {}).get("result", "")).strip()
    if result == "fail_is_not_develop_mode":
        return PreflightResult(
            False,
            "developer_mode_required",
            "当前不在高级开发者模式，请先用遥控器切换到高级开发者模式后再运行测试。",
            None,
            info_data,
        )
    if result and result != "success":
        return PreflightResult(False, "joint_state_failed", result, None, info_data)

    state = JointState.from_response(response)
    if len(state.q) < 16:
        return PreflightResult(False, "invalid_joint_state", "关节状态 q 少于 16 维", state, info_data)
    values = state.q[:16] + (state.dq[:16] if state.dq else [])
    if not all(math.isfinite(value) for value in values):
        return PreflightResult(False, "invalid_joint_state", "关节状态包含非有限数值", state, info_data)

    velocity_limit = float(config.get("thresholds", {}).get("stable_velocity_rad_s", 0.02))
    max_abs_dq = max(abs(value) for value in state.dq[:16]) if state.dq else 0.0
    if state.dq and max_abs_dq > velocity_limit:
        return PreflightResult(
            False,
            "robot_not_stationary",
            f"机器人关节速度未稳定: max_abs_dq={max_abs_dq:.5f} rad/s > {velocity_limit:.5f}",
            state,
            info_data,
            max_abs_dq,
        )

    return PreflightResult(True, "ok", "preflight passed", state, info_data, max_abs_dq)


def _latest_robot_info(client: Any) -> Dict[str, Any]:
    if not hasattr(client, "drain_notifications"):
        return {}
    infos = client.drain_notifications("notify_robot_info")
    return infos[-1] if infos else {}


def _response_timeout(config: Dict[str, Any]) -> float:
    return float(config.get("timeouts", {}).get("response_sec", 5.0))
