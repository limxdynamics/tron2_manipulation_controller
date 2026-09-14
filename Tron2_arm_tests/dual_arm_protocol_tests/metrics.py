from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List

from .trajectory import normalize_quat


@dataclass(frozen=True)
class JointError:
    max_abs: float
    rms: float
    per_joint: List[float]


@dataclass(frozen=True)
class PoseError:
    left_position: float
    right_position: float
    left_orientation_rad: float
    right_orientation_rad: float

    @property
    def max_position(self) -> float:
        return max(self.left_position, self.right_position)

    @property
    def max_orientation_rad(self) -> float:
        return max(self.left_orientation_rad, self.right_orientation_rad)


def joint_error(target: List[float], actual: List[float]) -> JointError:
    if len(target) != len(actual):
        raise ValueError("target and actual must have the same length")
    errors = [float(a - b) for a, b in zip(actual, target)]
    abs_errors = [abs(value) for value in errors]
    rms = math.sqrt(sum(value * value for value in errors) / len(errors)) if errors else 0.0
    return JointError(max(abs_errors) if abs_errors else 0.0, rms, errors)


def pose_error(target_pose: List[float], actual_pose: List[float]) -> PoseError:
    if len(target_pose) != 14 or len(actual_pose) != 14:
        raise ValueError("poses must contain 14 values")
    return PoseError(
        left_position=_euclidean(target_pose[0:3], actual_pose[0:3]),
        right_position=_euclidean(target_pose[7:10], actual_pose[7:10]),
        left_orientation_rad=quat_angle_rad(target_pose[3:7], actual_pose[3:7]),
        right_orientation_rad=quat_angle_rad(target_pose[10:14], actual_pose[10:14]),
    )


def quat_angle_rad(target: List[float], actual: List[float]) -> float:
    q_target = normalize_quat(target)
    q_actual = normalize_quat(actual)
    dot = abs(sum(a * b for a, b in zip(q_target, q_actual)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def summarize_scalar_errors(errors: Iterable[float]) -> Dict[str, float]:
    values = [float(value) for value in errors]
    if not values:
        return {"final": 0.0, "max": 0.0, "rms": 0.0}
    return {
        "final": values[-1],
        "max": max(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
    }


def _euclidean(target: List[float], actual: List[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(actual, target)))
