from __future__ import annotations

import time
import uuid
import math
from typing import Any, Dict, List, Optional

from .robot_model import load_default_robot_model


_ROBOT_MODEL = load_default_robot_model()

ARM_JOINT_UPPER_LIMITS = [
    limit.upper for limit in _ROBOT_MODEL.effective_limits[:14]
]
ARM_JOINT_LOWER_LIMITS = [
    limit.lower for limit in _ROBOT_MODEL.effective_limits[:14]
]
HEAD_LIMITS = [(limit.lower, limit.upper) for limit in _ROBOT_MODEL.head_limits]
HEAD_NAMES = [limit.name for limit in _ROBOT_MODEL.head_limits]

LEFT_WORKSPACE = {
    "x": (0.250, 0.732),
    "y": (-0.213, 0.900),
    "z": (-0.673, 0.500),
}
RIGHT_WORKSPACE = {
    "x": (0.250, 0.732),
    "y": (-0.900, 0.213),
    "z": (-0.673, 0.500),
}


def build_request(
    title: str,
    data: Optional[Dict[str, Any]] = None,
    *,
    accid: Optional[str],
    guid: Optional[str] = None,
    timestamp_ms: Optional[int] = None,
) -> Dict[str, Any]:
    return {
        "accid": accid,
        "title": title,
        "timestamp": timestamp_ms if timestamp_ms is not None else int(time.time() * 1000),
        "guid": guid or str(uuid.uuid4()),
        "data": data or {},
    }


def expected_response_title(request_title: str) -> str:
    if not request_title.startswith("request_"):
        raise ValueError("request title must start with 'request_'")
    return "response_" + request_title[len("request_") :]


def validate_command_payload(command_type: str, payload: Dict[str, Any]) -> List[str]:
    command = command_type.lower()
    errors = _validate_time(payload.get("time")) if command in {"movej", "moveh", "movep"} else []
    if command in {"movej", "servoj"}:
        values = payload.get("joint", payload.get("q"))
        errors.extend(_validate_arm_or_servo_joints(values, allow_head=command == "servoj"))
        if command == "servoj":
            errors.extend(_validate_filter_ratio(payload.get("filter_ratio", 1.0)))
        return errors
    if command == "moveh":
        errors.extend(_validate_head(payload.get("joint", payload.get("target_head"))))
        return errors
    if command == "movep":
        errors.extend(_validate_pose(payload.get("pos")))
        return errors
    if command == "servop":
        if "q" in payload:
            errors.extend(_validate_pose(payload.get("q")))
            errors.extend(_validate_positive_number(payload.get("t"), "t"))
        else:
            errors.extend(_validate_pose(payload.get("pos")))
        return errors
    return [f"unsupported command type: {command_type}"]


def _validate_arm_or_servo_joints(values: Any, *, allow_head: bool) -> List[str]:
    errors: List[str] = []
    if not isinstance(values, list):
        return ["joint/q must be a list"]
    allowed_lengths = {14, 16} if allow_head else {14}
    if len(values) not in allowed_lengths:
        return [f"joint/q must contain {sorted(allowed_lengths)} values, got {len(values)}"]
    for idx, value in enumerate(values[:14]):
        lower = ARM_JOINT_LOWER_LIMITS[idx]
        upper = ARM_JOINT_UPPER_LIMITS[idx]
        if not _is_finite_number(value):
            errors.append(f"joint[{idx}] must be finite")
        elif value < lower or value > upper:
            errors.append(f"joint[{idx}]={value} outside [{lower}, {upper}]")
    if len(values) == 16:
        errors.extend(_validate_head(values[14:16]))
    return errors


def _validate_head(values: Any) -> List[str]:
    errors: List[str] = []
    if not isinstance(values, list):
        return ["head joint target must be a list"]
    if len(values) != 2:
        return [f"head joint target must contain 2 values, got {len(values)}"]
    for idx, value in enumerate(values):
        lower, upper = HEAD_LIMITS[idx]
        name = HEAD_NAMES[idx]
        if not _is_finite_number(value):
            errors.append(f"{name} must be finite")
        elif value < lower or value > upper:
            errors.append(f"{name}={value} outside [{lower}, {upper}]")
    return errors


def _validate_pose(values: Any) -> List[str]:
    errors: List[str] = []
    if not isinstance(values, list):
        return ["pos must be a list"]
    if len(values) != 14:
        return [f"pos must contain 14 values, got {len(values)}"]
    for idx, value in enumerate(values):
        if not _is_finite_number(value):
            errors.append(f"pos[{idx}] must be finite")
    errors.extend(_validate_xyz(values[0:3], LEFT_WORKSPACE, "left"))
    errors.extend(_validate_xyz(values[7:10], RIGHT_WORKSPACE, "right"))
    errors.extend(_validate_quat(values[3:7], "left_quat"))
    errors.extend(_validate_quat(values[10:14], "right_quat"))
    return errors


def _validate_xyz(values: List[float], bounds: Dict[str, tuple], arm: str) -> List[str]:
    errors: List[str] = []
    for idx, axis in enumerate(("x", "y", "z")):
        lower, upper = bounds[axis]
        value = values[idx]
        if value < lower or value > upper:
            errors.append(f"{arm}.{axis}={value} outside [{lower}, {upper}]")
    return errors


def _validate_time(value: Any) -> List[str]:
    return _validate_positive_number(value, "time")


def _validate_positive_number(value: Any, name: str) -> List[str]:
    if not _is_finite_number(value) or float(value) <= 0:
        return [f"{name} must be a positive finite number"]
    return []


def _validate_filter_ratio(value: Any) -> List[str]:
    if not _is_finite_number(value):
        return ["filter_ratio must be finite"]
    if float(value) < 0.0 or float(value) > 1.0:
        return ["filter_ratio must be in [0, 1]"]
    return []


def _validate_quat(values: List[float], name: str) -> List[str]:
    norm_sq = sum(float(value) * float(value) for value in values if _is_finite_number(value))
    if norm_sq <= 0.0:
        return [f"{name} must not be zero"]
    return []


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))
