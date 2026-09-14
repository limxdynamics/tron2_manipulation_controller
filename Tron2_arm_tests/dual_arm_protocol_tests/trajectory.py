from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class VectorSample:
    t: float
    values: List[float]


def interpolate_vectors(
    start: List[float],
    target: List[float],
    *,
    duration_sec: float,
    rate_hz: float,
) -> List[VectorSample]:
    if len(start) != len(target):
        raise ValueError("start and target must have the same length")
    if duration_sec <= 0:
        return [VectorSample(0.0, list(target))]
    step_count = max(1, int(round(duration_sec * rate_hz)))
    samples = []
    for idx in range(step_count + 1):
        ratio = idx / step_count
        t = duration_sec * ratio
        values = [
            float(a + (b - a) * ratio)
            for a, b in zip(start, target)
        ]
        samples.append(VectorSample(round(t, 10), values))
    return samples


def nearest_reference(samples: List[VectorSample], timestamp_sec: float) -> VectorSample:
    if not samples:
        raise ValueError("samples must not be empty")
    times = [sample.t for sample in samples]
    idx = bisect.bisect_left(times, timestamp_sec)
    if idx == 0:
        return samples[0]
    if idx >= len(samples):
        return samples[-1]
    before = samples[idx - 1]
    after = samples[idx]
    if abs(timestamp_sec - before.t) <= abs(after.t - timestamp_sec):
        return before
    return after


def normalize_quat(quat: List[float]) -> List[float]:
    norm = math.sqrt(sum(value * value for value in quat))
    if norm == 0:
        raise ValueError("quaternion norm must not be zero")
    return [value / norm for value in quat]


def slerp_quat(start: List[float], target: List[float], ratio: float) -> List[float]:
    q0 = normalize_quat(start)
    q1 = normalize_quat(target)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = [-value for value in q1]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quat([a + ratio * (b - a) for a, b in zip(q0, q1)])
    theta_0 = math.acos(dot)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return [s0 * a + s1 * b for a, b in zip(q0, q1)]


def interpolate_pose(
    start_pose: List[float],
    target_pose: List[float],
    *,
    duration_sec: float,
    rate_hz: float,
) -> List[VectorSample]:
    if len(start_pose) != 14 or len(target_pose) != 14:
        raise ValueError("poses must contain 14 values")
    step_count = max(1, int(round(duration_sec * rate_hz)))
    samples = []
    for idx in range(step_count + 1):
        ratio = idx / step_count
        left_xyz = _lerp_list(start_pose[0:3], target_pose[0:3], ratio)
        right_xyz = _lerp_list(start_pose[7:10], target_pose[7:10], ratio)
        values = (
            left_xyz
            + slerp_quat(start_pose[3:7], target_pose[3:7], ratio)
            + right_xyz
            + slerp_quat(start_pose[10:14], target_pose[10:14], ratio)
        )
        samples.append(VectorSample(round(duration_sec * ratio, 10), values))
    return samples


def _lerp_list(start: List[float], target: List[float], ratio: float) -> List[float]:
    return [float(a + (b - a) * ratio) for a, b in zip(start, target)]
