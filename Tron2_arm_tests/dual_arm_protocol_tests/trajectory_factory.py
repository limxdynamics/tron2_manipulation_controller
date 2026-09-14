from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple
import math

from .robot_model import DEFAULT_HOME_Q16, RobotModel, load_default_robot_model
from .trajectory import VectorSample, slerp_quat


DEFAULT_REFERENCE_POSE: Tuple[float, ...] = (
    0.50,
    0.28,
    -0.25,
    0.706775,
    0.006158,
    -0.707391,
    -0.005416,
    0.50,
    -0.28,
    -0.25,
    0.706775,
    -0.006158,
    -0.707391,
    0.005416,
)


@dataclass(frozen=True)
class Waypoint:
    name: str
    values: List[float]


@dataclass(frozen=True)
class TrajectoryPattern:
    name: str
    waypoints: List[List[float]]


@dataclass(frozen=True)
class LoopTrajectory:
    samples: List[VectorSample]
    planned_duration_sec: float
    completed_segments: int
    completed_cycles: int


def smoothstep5(ratio: float) -> float:
    x = max(0.0, min(1.0, float(ratio)))
    return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x)


def generate_movej_waypoints(
    model: RobotModel,
    *,
    groups: Optional[Iterable[str]] = None,
    home: Optional[List[float]] = None,
) -> List[Waypoint]:
    enabled = set(
        ["single_joint", "mirror", "opposite", "coordinated"]
        if groups is None
        else groups
    )
    center = list(home or DEFAULT_HOME_Q16)
    safe = model.safe_limits(home=center)
    waypoints: List[Waypoint] = []

    if "single_joint" in enabled:
        for idx in (0, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13):
            values = list(center)
            values[idx] = _towards_upper(center[idx], safe[idx])
            waypoints.append(Waypoint(f"single_q{idx}_plus", values))

    if "mirror" in enabled:
        values = list(center)
        for left, right in ((0, 7), (2, 9), (3, 10), (5, 12), (6, 13)):
            delta = min(safe[left].upper - center[left], safe[right].upper - center[right], 0.18)
            values[left] = center[left] + delta
            values[right] = center[right] + delta
        waypoints.append(Waypoint("dual_arm_mirror_plus", values))

    if "opposite" in enabled:
        values = list(center)
        for left, right in ((1, 8), (2, 9), (4, 11)):
            delta = min(safe[left].upper - center[left], center[right] - safe[right].lower, 0.18)
            values[left] = center[left] + delta
            values[right] = center[right] - delta
        waypoints.append(Waypoint("dual_arm_opposite", values))

    if "coordinated" in enabled:
        values = list(center)
        for idx, sign in ((0, 1), (3, -1), (7, 1), (10, -1), (14, 1), (15, -1)):
            candidate = center[idx] + sign * 0.18
            values[idx] = max(safe[idx].lower, min(safe[idx].upper, candidate))
        waypoints.append(Waypoint("arm_head_coordinated", values))

    return waypoints


def generate_movej_patterns(model: RobotModel) -> List[TrajectoryPattern]:
    """Build deterministic, safe joint-space patterns suitable for repetition."""
    center = list(DEFAULT_HOME_Q16)
    safe = model.safe_limits(home=center)
    specifications = [
        ("dual_shoulder_elbow_lift", {0: 0.16, 3: 0.18, 7: 0.12, 10: 0.18}),
        ("wrist_head_coordination", {5: 0.14, 6: -0.12, 12: -0.14, 13: 0.12, 14: 0.08, 15: -0.12}),
        ("dual_mirror_shoulder", {0: 0.16, 7: 0.16}),
        ("dual_mirror_wrist", {5: 0.14, 6: -0.12, 12: 0.14, 13: -0.12}),
        ("dual_opposite_shoulder", {0: 0.16, 7: -0.16}),
        ("dual_opposite_wrist", {5: 0.14, 6: 0.12, 12: -0.14, 13: -0.12}),
        (
            "coordinated_reach",
            {0: 0.14, 2: 0.18, 3: 0.20, 7: 0.14, 9: -0.18, 10: 0.20},
        ),
        (
            "coordinated_fold",
            {1: 0.12, 3: -0.18, 4: 0.14, 8: -0.12, 10: -0.18, 11: -0.14},
        ),
        ("arm_head_nod", {3: 0.12, 10: 0.12, 14: 0.16}),
        ("arm_head_scan", {0: 0.12, 7: -0.12, 14: 0.08, 15: 0.20}),
    ]
    return [
        TrajectoryPattern(name, _symmetric_joint_pair(center, safe, deltas))
        for name, deltas in specifications
    ]


def generate_moveh_patterns(model: RobotModel) -> List[TrajectoryPattern]:
    """Build deterministic head pitch/yaw patterns inside the model safe limits."""
    pitch_limit, yaw_limit = model.safe_limits()[14:16]
    pitch_center = (pitch_limit.lower + pitch_limit.upper) / 2.0
    yaw_center = (yaw_limit.lower + yaw_limit.upper) / 2.0
    pitch_radius = min(pitch_center - pitch_limit.lower, pitch_limit.upper - pitch_center)
    yaw_radius = min(yaw_center - yaw_limit.lower, yaw_limit.upper - yaw_center)
    diagonal = 0.60 * min(pitch_radius, yaw_radius)

    def point(pitch_fraction: float, yaw_fraction: float) -> List[float]:
        return [
            pitch_center + pitch_fraction * pitch_radius,
            yaw_center + yaw_fraction * yaw_radius,
        ]

    return [
        TrajectoryPattern("pitch_axis", [point(0.72, 0.0), point(-0.72, 0.0)]),
        TrajectoryPattern("yaw_axis", [point(0.0, 0.72), point(0.0, -0.72)]),
        TrajectoryPattern(
            "diagonal_same",
            [
                [pitch_center + diagonal, yaw_center + diagonal],
                [pitch_center - diagonal, yaw_center - diagonal],
            ],
        ),
        TrajectoryPattern(
            "diagonal_opposite",
            [
                [pitch_center + diagonal, yaw_center - diagonal],
                [pitch_center - diagonal, yaw_center + diagonal],
            ],
        ),
        TrajectoryPattern(
            "pitch_offset_positive",
            [point(0.80, -0.12), point(0.30, 0.12)],
        ),
        TrajectoryPattern(
            "pitch_offset_negative",
            [point(-0.80, 0.12), point(-0.30, -0.12)],
        ),
        TrajectoryPattern(
            "yaw_offset_positive",
            [point(-0.20, 0.80), point(0.20, 0.35)],
        ),
        TrajectoryPattern(
            "yaw_offset_negative",
            [point(0.20, -0.80), point(-0.20, -0.35)],
        ),
        TrajectoryPattern(
            "asymmetric_positive",
            [point(0.35, 0.80), point(-0.35, -0.55)],
        ),
        TrajectoryPattern(
            "asymmetric_negative",
            [point(0.75, -0.30), point(-0.45, 0.65)],
        ),
    ]


def generate_movep_patterns(reference_pose: List[float]) -> List[TrajectoryPattern]:
    """Build conservative Cartesian patterns around a validated pose.

    Validation here covers only finite geometry and unit quaternions. Whether a
    generated pose is reachable must still be established against the live
    robot's IK and collision environment.
    """
    reference = _validated_pose(reference_pose)
    specifications = [
        ("both_x", ((0.02, 0.0, 0.0), (0.02, 0.0, 0.0), None, None)),
        ("both_y", ((0.0, 0.02, 0.0), (0.0, 0.02, 0.0), None, None)),
        ("both_z", ((0.0, 0.0, 0.02), (0.0, 0.0, 0.02), None, None)),
        ("mirror_x", ((0.02, 0.0, 0.0), (-0.02, 0.0, 0.0), None, None)),
        ("mirror_y", ((0.0, 0.02, 0.0), (0.0, -0.02, 0.0), None, None)),
        ("opposite_z", ((0.0, 0.0, 0.02), (0.0, 0.0, -0.02), None, None)),
        ("left_roll", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 0.10), None)),
        ("right_pitch", ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None, (1, 0.10))),
        (
            "dual_yaw_opposite",
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (2, 0.09), (2, -0.09)),
        ),
        (
            "asymmetric_xyz_orientation",
            ((0.015, -0.010, 0.012), (-0.010, 0.016, -0.008), (1, 0.07), (0, -0.06)),
        ),
    ]
    patterns: List[TrajectoryPattern] = []
    for name, (left_xyz, right_xyz, left_rotation, right_rotation) in specifications:
        positive = _offset_pose(
            reference,
            left_xyz,
            right_xyz,
            left_rotation,
            right_rotation,
            sign=1.0,
        )
        negative = _offset_pose(
            reference,
            left_xyz,
            right_xyz,
            left_rotation,
            right_rotation,
            sign=-1.0,
        )
        patterns.append(TrajectoryPattern(name, [positive, negative]))
    return patterns


def resolve_case_waypoints(
    case: Dict[str, Any],
    *,
    reference_pose: Optional[List[float]] = None,
    model: Optional[RobotModel] = None,
) -> List[List[float]]:
    """Resolve explicit or named case waypoints through one shared catalog."""
    case_type = str(case["type"])
    raw_waypoints = case.get("waypoints")
    robot_model = model or load_default_robot_model()

    if raw_waypoints is None:
        pattern_name = str(case.get("pattern", ""))
        if case_type in {"moveJ", "servoJ"}:
            patterns = generate_movej_patterns(robot_model)
        elif case_type == "moveH":
            patterns = generate_moveh_patterns(robot_model)
        elif case_type in {"moveP", "servoP"}:
            reference = (
                list(reference_pose)
                if reference_pose is not None
                else list(
                    case.get("reference_pose", DEFAULT_REFERENCE_POSE)
                )
            )
            patterns = generate_movep_patterns(reference)
        else:
            raise ValueError(f"unsupported waypoint case type: {case_type}")
        raw_waypoints = _named_pattern_waypoints(
            patterns,
            pattern_name,
            case_type,
        )

    waypoints: List[List[float]] = []
    for raw in raw_waypoints:
        if case_type == "moveJ":
            values = _float_values(raw)
            if len(values) not in (14, 16):
                raise ValueError("moveJ waypoints must contain 14 or 16 values")
            waypoints.append(values[:14])
        elif case_type == "moveH":
            values = _float_values(raw)
            if len(values) == 16:
                values = values[14:16]
            if len(values) != 2:
                raise ValueError("moveH waypoints must contain 2 or 16 values")
            waypoints.append(values)
        elif case_type == "moveP":
            values = _float_values(raw)
            if len(values) != 14:
                raise ValueError("moveP waypoints must contain 14 values")
            waypoints.append(values)
        elif case_type in {"servoJ", "servoP"}:
            values = _float_values(raw)
            expected_length = 16 if case_type == "servoJ" else 14
            if len(values) != expected_length:
                raise ValueError(
                    f"{case_type} waypoints must contain {expected_length} values"
                )
            waypoints.append(values)
        else:
            raise ValueError(f"unsupported waypoint case type: {case_type}")

    if not waypoints:
        raise ValueError("waypoints must not be empty")
    return waypoints


def _named_pattern_waypoints(
    patterns: Iterable[TrajectoryPattern],
    pattern_name: str,
    case_type: str,
) -> List[List[float]]:
    by_name = {pattern.name: pattern.waypoints for pattern in patterns}
    if pattern_name not in by_name:
        raise ValueError(f"unknown {case_type} pattern: {pattern_name}")
    return by_name[pattern_name]


def _float_values(values: Iterable[Any]) -> List[float]:
    return [float(value) for value in values]


def build_servoj_trajectory(
    start: List[float],
    waypoints: Iterable[List[float]],
    *,
    model: RobotModel,
    rate_hz: float = 300.0,
    max_velocity_rad_s: float = 0.5,
    min_segment_sec: float = 1.0,
) -> List[VectorSample]:
    if rate_hz <= 0:
        raise ValueError("rate_hz must be positive")
    if max_velocity_rad_s <= 0:
        raise ValueError("max_velocity_rad_s must be positive")
    current = list(start)
    if len(current) != 16:
        raise ValueError("start must contain 16 values")
    errors = model.validate_safe(current)
    if errors:
        raise ValueError("; ".join(errors))

    samples: List[VectorSample] = [VectorSample(0.0, list(current))]
    current_time = 0.0
    for waypoint in waypoints:
        target = list(waypoint)
        if len(target) != 16:
            raise ValueError("waypoints must contain 16 values")
        errors = model.validate_safe(target)
        if errors:
            raise ValueError("; ".join(errors))
        max_delta = max(abs(b - a) for a, b in zip(current, target))
        duration = max(min_segment_sec, max_delta / max_velocity_rad_s)
        step_count = max(1, int(math.ceil(duration * rate_hz)))
        for idx in range(1, step_count + 1):
            ratio = smoothstep5(idx / step_count)
            values = [a + (b - a) * ratio for a, b in zip(current, target)]
            t = current_time + idx / rate_hz
            samples.append(VectorSample(round(t, 10), [float(v) for v in values]))
        current_time = samples[-1].t
        current = target
    return samples


def build_servoj_loop_trajectory(
    start: List[float],
    waypoints: Iterable[List[float]],
    *,
    model: RobotModel,
    active_duration_sec: float,
    segment_duration_sec: float,
    rate_hz: float = 300.0,
) -> LoopTrajectory:
    points = [list(waypoint) for waypoint in waypoints]
    current = list(start)
    if len(current) != 16:
        raise ValueError("start must contain 16 values")
    errors = model.validate_safe(current)
    if errors:
        raise ValueError("; ".join(errors))
    for waypoint in points:
        if len(waypoint) != 16:
            raise ValueError("waypoints must contain 16 values")
        errors = model.validate_safe(waypoint)
        if errors:
            raise ValueError("; ".join(errors))
    return _build_loop_trajectory(
        current,
        points,
        active_duration_sec=active_duration_sec,
        segment_duration_sec=segment_duration_sec,
        rate_hz=rate_hz,
        pose=False,
    )


def build_servop_loop_trajectory(
    start_pose: List[float],
    waypoints: Iterable[List[float]],
    *,
    active_duration_sec: float,
    segment_duration_sec: float,
    rate_hz: float = 300.0,
) -> LoopTrajectory:
    start = _validated_pose(start_pose)
    points = [_validated_pose(list(waypoint)) for waypoint in waypoints]
    return _build_loop_trajectory(
        start,
        points,
        active_duration_sec=active_duration_sec,
        segment_duration_sec=segment_duration_sec,
        rate_hz=rate_hz,
        pose=True,
    )


def build_servoj_target_trajectory(
    start: List[float],
    target: List[float],
    *,
    model: RobotModel,
    duration_sec: float,
    rate_hz: float = 300.0,
) -> LoopTrajectory:
    current = list(start)
    destination = list(target)
    if len(current) != 16 or len(destination) != 16:
        raise ValueError("start and target must contain 16 values")
    for values in (current, destination):
        errors = model.validate_safe(values)
        if errors:
            raise ValueError("; ".join(errors))
    return _build_target_trajectory(
        current,
        destination,
        duration_sec=duration_sec,
        rate_hz=rate_hz,
        pose=False,
    )


def build_servop_target_trajectory(
    start_pose: List[float],
    target_pose: List[float],
    *,
    duration_sec: float,
    rate_hz: float = 300.0,
) -> LoopTrajectory:
    return _build_target_trajectory(
        _validated_pose(start_pose),
        _validated_pose(target_pose),
        duration_sec=duration_sec,
        rate_hz=rate_hz,
        pose=True,
    )


def _build_target_trajectory(
    start: List[float],
    target: List[float],
    *,
    duration_sec: float,
    rate_hz: float,
    pose: bool,
) -> LoopTrajectory:
    duration = float(duration_sec)
    rate = float(rate_hz)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration_sec must be a finite positive number")
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("rate_hz must be a finite positive number")

    samples = [VectorSample(0.0, list(start))]
    step_count = max(1, int(round(duration * rate)))
    for index in range(1, step_count + 1):
        progress = smoothstep5(index / step_count)
        if pose:
            values = _interpolate_pose_smoothstep(start, target, progress)
        else:
            values = [
                float(a + (b - a) * progress)
                for a, b in zip(start, target)
            ]
        timestamp = duration * index / step_count
        samples.append(VectorSample(round(timestamp, 10), values))

    return LoopTrajectory(
        samples=samples,
        planned_duration_sec=duration,
        completed_segments=1,
        completed_cycles=0,
    )


def _build_loop_trajectory(
    start: List[float],
    waypoints: List[List[float]],
    *,
    active_duration_sec: float,
    segment_duration_sec: float,
    rate_hz: float,
    pose: bool,
) -> LoopTrajectory:
    active_duration = float(active_duration_sec)
    segment_duration = float(segment_duration_sec)
    rate = float(rate_hz)
    if not math.isfinite(active_duration) or active_duration <= 0.0:
        raise ValueError("active_duration_sec must be a finite positive number")
    if (
        not math.isfinite(segment_duration)
        or segment_duration < 1.0
        or segment_duration > 5.0
    ):
        raise ValueError("segment_duration_sec must be between 1 and 5 seconds")
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("rate_hz must be a finite positive number")
    if not waypoints:
        raise ValueError("waypoints must not be empty")

    samples = [VectorSample(0.0, list(start))]
    current = list(start)
    elapsed = 0.0
    completed_segments = 0
    step_count = max(1, int(round(segment_duration * rate)))
    while elapsed < active_duration:
        target = waypoints[completed_segments % len(waypoints)]
        for index in range(1, step_count + 1):
            progress = smoothstep5(index / step_count)
            if pose:
                values = _interpolate_pose_smoothstep(current, target, progress)
            else:
                values = [
                    float(a + (b - a) * progress)
                    for a, b in zip(current, target)
                ]
            timestamp = elapsed + segment_duration * index / step_count
            samples.append(VectorSample(round(timestamp, 10), values))
        elapsed += segment_duration
        completed_segments += 1
        current = list(target)

    return LoopTrajectory(
        samples=samples,
        planned_duration_sec=elapsed,
        completed_segments=completed_segments,
        completed_cycles=completed_segments // len(waypoints),
    )


def _interpolate_pose_smoothstep(
    start_pose: List[float],
    target_pose: List[float],
    progress: float,
) -> List[float]:
    return (
        [
            float(a + (b - a) * progress)
            for a, b in zip(start_pose[0:3], target_pose[0:3])
        ]
        + slerp_quat(start_pose[3:7], target_pose[3:7], progress)
        + [
            float(a + (b - a) * progress)
            for a, b in zip(start_pose[7:10], target_pose[7:10])
        ]
        + slerp_quat(start_pose[10:14], target_pose[10:14], progress)
    )


def _towards_upper(home_value: float, limit) -> float:
    return min(limit.upper, home_value + min(0.2, max(0.0, limit.upper - home_value)))


def _symmetric_joint_pair(center, safe_limits, deltas) -> List[List[float]]:
    scale = 1.0
    for index, delta in deltas.items():
        if delta == 0.0:
            continue
        available = min(
            safe_limits[index].upper - center[index],
            center[index] - safe_limits[index].lower,
        )
        scale = min(scale, available / abs(delta))

    positive = list(center)
    negative = list(center)
    for index, delta in deltas.items():
        positive[index] += delta * scale
        negative[index] -= delta * scale
    return [positive, negative]


def _validated_pose(reference_pose: List[float]) -> List[float]:
    reference = list(reference_pose)
    if len(reference) != 14:
        raise ValueError("reference_pose must contain 14 values")
    if any(
        not isinstance(value, (int, float)) or not math.isfinite(float(value))
        for value in reference
    ):
        raise ValueError("reference_pose values must be finite")
    reference = [float(value) for value in reference]
    for start in (3, 10):
        norm = math.sqrt(sum(value * value for value in reference[start : start + 4]))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("reference_pose quaternions must be normalized")
    return reference


def _offset_pose(
    reference,
    left_xyz,
    right_xyz,
    left_rotation,
    right_rotation,
    *,
    sign,
) -> List[float]:
    left_position = [
        value + sign * delta for value, delta in zip(reference[0:3], left_xyz)
    ]
    right_position = [
        value + sign * delta for value, delta in zip(reference[7:10], right_xyz)
    ]
    left_quaternion = _rotated_quaternion(reference[3:7], left_rotation, sign)
    right_quaternion = _rotated_quaternion(reference[10:14], right_rotation, sign)
    return left_position + left_quaternion + right_position + right_quaternion


def _rotated_quaternion(quaternion, rotation, sign) -> List[float]:
    if rotation is None:
        return list(quaternion)
    axis, angle = rotation
    half_angle = sign * angle / 2.0
    delta = [math.cos(half_angle), 0.0, 0.0, 0.0]
    delta[axis + 1] = math.sin(half_angle)
    w1, x1, y1, z1 = delta
    w2, x2, y2, z2 = quaternion
    result = [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]
    norm = math.sqrt(sum(value * value for value in result))
    return [value / norm for value in result]
