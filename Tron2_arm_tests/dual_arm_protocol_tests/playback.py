from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List
import math

from .robot_model import RobotModel
from .trajectory import VectorSample


FILE_COL_TO_Q_IDX: Dict[int, int] = {
    3: 0,
    4: 1,
    5: 2,
    6: 3,
    7: 4,
    8: 5,
    9: 6,
    11: 7,
    12: 8,
    13: 9,
    14: 10,
    15: 11,
    16: 12,
    17: 13,
}

FILE_HEADER_SCHEMA = {
    0: {"episode"},
    1: {"step", "frame"},
    2: {"time", "timestamp"},
    3: {"abad_L"},
    4: {"hip_L"},
    5: {"yaw_L"},
    6: {"knee_L"},
    7: {"wrist_yaw_L"},
    8: {"wrist_pitch_L"},
    9: {"wrist_roll_L"},
    11: {"abad_R"},
    12: {"hip_R"},
    13: {"yaw_R"},
    14: {"knee_R"},
    15: {"wrist_yaw_R"},
    16: {"wrist_pitch_R"},
    17: {"wrist_roll_R"},
}

MAX_HARD_LIMIT_TOLERANCE_RAD = 0.001

@dataclass(frozen=True)
class VlaClampStats:
    clamped_value_count: int = 0
    clamped_frame_count: int = 0
    max_clamp_correction_rad: float = 0.0

    def as_dict(self) -> Dict[str, float]:
        return {
            "clamped_value_count": self.clamped_value_count,
            "clamped_frame_count": self.clamped_frame_count,
            "max_clamp_correction_rad": self.max_clamp_correction_rad,
        }


@dataclass(frozen=True)
class VlaLoadResult:
    frames: List[VectorSample]
    stats: VlaClampStats


@dataclass(frozen=True)
class ParsedVlaRow:
    t: float
    q: List[float]
    clamped_value_count: int
    max_clamp_correction_rad: float


def generate_preroll(
    current_q16: List[float],
    first_q16: List[float],
    *,
    rate_hz: float = 300.0,
    max_velocity_rad_s: float = 0.5,
) -> List[VectorSample]:
    if len(current_q16) != 16 or len(first_q16) != 16:
        raise ValueError("current_q16 and first_q16 must contain 16 values")
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("rate_hz must be a finite positive number")
    if not math.isfinite(max_velocity_rad_s) or max_velocity_rad_s <= 0.0:
        raise ValueError("max_velocity_rad_s must be a finite positive number")
    if not all(math.isfinite(value) for value in current_q16 + first_q16):
        raise ValueError("joint values must be finite")

    max_delta = max(
        abs(target - current)
        for current, target in zip(current_q16, first_q16)
    )
    if max_delta == 0.0:
        return [VectorSample(0.0, [float(value) for value in current_q16])]

    # max(d/du (6u^5 - 15u^4 + 10u^3)) = 1.875 at u=0.5.
    minimum_duration = 1.875 * max_delta / max_velocity_rad_s
    step_count = max(1, int(math.ceil(minimum_duration * rate_hz)))
    samples: List[VectorSample] = []
    for step in range(step_count + 1):
        ratio = step / step_count
        blend = 6.0 * ratio**5 - 15.0 * ratio**4 + 10.0 * ratio**3
        values = [
            float(current + (target - current) * blend)
            for current, target in zip(current_q16, first_q16)
        ]
        samples.append(VectorSample(step / rate_hz, values))
    return samples


def load_vla_states(
    path: Path,
    initial_q16: List[float],
    *,
    model: RobotModel,
    max_joint_step_rad: float = 0.25,
    max_source_velocity_rad_s: float = 3.0,
    hard_limit_tolerance_rad: float = 0.0,
) -> List[VectorSample]:
    """Backward-compatible frame-only wrapper around the statistics API."""
    return load_vla_states_with_stats(
        path,
        initial_q16,
        model=model,
        max_joint_step_rad=max_joint_step_rad,
        max_source_velocity_rad_s=max_source_velocity_rad_s,
        hard_limit_tolerance_rad=hard_limit_tolerance_rad,
    ).frames


def load_vla_states_with_stats(
    path: Path,
    initial_q16: List[float],
    *,
    model: RobotModel,
    max_joint_step_rad: float = 0.25,
    max_source_velocity_rad_s: float = 3.0,
    hard_limit_tolerance_rad: float = 0.0,
) -> VlaLoadResult:
    """Load and validate a recorded VLA trajectory without reshaping it.

    Generated Move/Servo waypoints are constrained to the home-centered safe
    envelope. Recorded VLA data must preserve its original motion, so playback
    instead enforces physical hard limits. A configured, VLA-only tolerance may
    clamp tiny recording overshoots to the exact hard limit; timestamps remain
    unchanged, and joint-step/velocity checks run on the clamped values.
    """
    if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0.0:
        raise ValueError(
            "max_joint_step_rad must be a finite positive number"
        )
    if (
        not math.isfinite(max_source_velocity_rad_s)
        or max_source_velocity_rad_s <= 0.0
    ):
        raise ValueError(
            "max_source_velocity_rad_s must be a finite positive number"
        )
    if (
        not math.isfinite(hard_limit_tolerance_rad)
        or hard_limit_tolerance_rad < 0.0
        or hard_limit_tolerance_rad > MAX_HARD_LIMIT_TOLERANCE_RAD
    ):
        raise ValueError(
            "hard_limit_tolerance_rad must be between 0 and 0.001"
        )

    frames: List[VectorSample] = []
    clamped_value_count = 0
    clamped_frame_count = 0
    max_clamp_correction = 0.0
    with path.open("r", encoding="utf-8") as handle:
        first_line = handle.readline()
        if not first_line:
            raise ValueError("empty states file")
        first_parts = first_line.split()
        pending_rows = []
        if _looks_like_data_row(first_parts):
            pending_rows.append((1, first_line))
        else:
            _validate_header_parts(first_parts)
        previous_t = None
        previous_q = None
        for line_number, line in pending_rows:
            row = _parse_vla_data_row(
                line_number,
                line,
                initial_q16,
                model=model,
                hard_limit_tolerance_rad=hard_limit_tolerance_rad,
            )
            clamped_value_count += row.clamped_value_count
            if row.clamped_value_count:
                clamped_frame_count += 1
                max_clamp_correction = max(
                    max_clamp_correction,
                    row.max_clamp_correction_rad,
                )
            frames.append(VectorSample(row.t, row.q))
            previous_t = row.t
            previous_q = row.q
        for line_number, line in enumerate(handle, start=2):
            parts = line.split()
            if not parts:
                continue
            if len(parts) < 19:
                raise ValueError(f"line {line_number}: expected at least 19 columns")
            row = _parse_vla_data_row(
                line_number,
                line,
                initial_q16,
                model=model,
                hard_limit_tolerance_rad=hard_limit_tolerance_rad,
            )
            if previous_t is not None and row.t <= previous_t:
                raise ValueError(f"line {line_number}: timestamp must be monotonic")
            clamped_value_count += row.clamped_value_count
            if row.clamped_value_count:
                clamped_frame_count += 1
                max_clamp_correction = max(
                    max_clamp_correction,
                    row.max_clamp_correction_rad,
                )
            if previous_q is not None:
                deltas = [
                    abs(value - old)
                    for value, old in zip(row.q, previous_q)
                ]
                max_step = max(deltas)
                if max_step > max_joint_step_rad:
                    joint_index = deltas.index(max_step)
                    raise ValueError(
                        f"line {line_number}: joint[{joint_index}] step "
                        f"{max_step:.3f} rad too large"
                    )
                delta_t = row.t - previous_t
                for joint_index, delta in enumerate(deltas):
                    velocity = delta / delta_t
                    if velocity > max_source_velocity_rad_s:
                        raise ValueError(
                            f"line {line_number}: joint[{joint_index}] velocity "
                            f"{velocity:.3f} rad/s exceeds "
                            f"max_source_velocity_rad_s="
                            f"{max_source_velocity_rad_s:.3f}"
                        )
            frames.append(VectorSample(row.t, row.q))
            previous_t = row.t
            previous_q = row.q
    if not frames:
        raise ValueError("states file contains no frames")
    return VlaLoadResult(
        frames=frames,
        stats=VlaClampStats(
            clamped_value_count=clamped_value_count,
            clamped_frame_count=clamped_frame_count,
            max_clamp_correction_rad=max_clamp_correction,
        ),
    )


def _validate_header_parts(header_parts: List[str]) -> None:
    invalid_columns = [
        index
        for index, accepted in FILE_HEADER_SCHEMA.items()
        if index >= len(header_parts) or header_parts[index] not in accepted
    ]
    if invalid_columns:
        raise ValueError(
            "states file header does not match expected schema "
            f"at columns {invalid_columns}"
        )


def _looks_like_data_row(parts: List[str]) -> bool:
    if len(parts) < 19:
        return False
    try:
        [float(part) for part in parts[:19]]
    except ValueError:
        return False
    return True


def _parse_vla_data_row(
    line_number: int,
    line: str,
    initial_q16: List[float],
    *,
    model: RobotModel,
    hard_limit_tolerance_rad: float,
) -> tuple:
    parts = line.split()
    try:
        t = float(parts[2])
        q = list(initial_q16)
        for column, q_idx in FILE_COL_TO_Q_IDX.items():
            q[q_idx] = float(parts[column])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"line {line_number}: non-numeric value") from exc
    if not math.isfinite(t):
        raise ValueError(
            f"line {line_number}: timestamp must be finite"
        )
    q, frame_clamp_count, frame_max_correction = _clamp_hard_limits(
        q,
        model=model,
        line_number=line_number,
        tolerance_rad=hard_limit_tolerance_rad,
    )
    effective_errors = model.validate_effective(q)
    if effective_errors:
        raise ValueError(
            f"line {line_number}: {effective_errors[0]}; "
            "controller effective limit violation"
        )
    return ParsedVlaRow(t, q, frame_clamp_count, frame_max_correction)


def _clamp_hard_limits(
    values: List[float],
    *,
    model: RobotModel,
    line_number: int,
    tolerance_rad: float,
) -> tuple:
    if len(values) != len(model.limits):
        raise ValueError(
            f"line {line_number}: expected {len(model.limits)} joint values, "
            f"got {len(values)}"
        )
    clamped = list(values)
    count = 0
    maximum = 0.0
    for index, (value, limit) in enumerate(zip(values, model.limits)):
        if not isinstance(value, (int, float)) or not math.isfinite(
            float(value)
        ):
            raise ValueError(
                f"line {line_number}: joint[{index}] must be finite"
            )
        bound = None
        side = ""
        if value < limit.lower:
            bound = limit.lower
            side = "lower"
        elif value > limit.upper:
            bound = limit.upper
            side = "upper"
        if bound is None:
            continue
        correction = abs(float(value) - bound)
        if correction > tolerance_rad and not math.isclose(
            correction,
            tolerance_rad,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                f"line {line_number}: joint[{index}]={float(value):.6f} "
                f"exceeds hard {side}={bound:.6f}; "
                f"excess={correction:.6f} rad exceeds "
                f"hard_limit_tolerance_rad={tolerance_rad:.6f}"
            )
        clamped[index] = bound
        count += 1
        maximum = max(maximum, correction)
    return clamped, count, maximum


def resample_frames(frames: List[VectorSample], *, rate_hz: float) -> List[VectorSample]:
    if not frames:
        raise ValueError("frames must not be empty")
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("rate_hz must be a finite positive number")
    if len(frames) == 1:
        return [VectorSample(0.0, list(frames[0].values))]

    duration = frames[-1].t - frames[0].t
    if duration <= 0:
        raise ValueError("frame duration must be positive")
    step_count = max(1, int(math.ceil(duration * rate_hz)))
    samples: List[VectorSample] = []
    source_index = 0
    for step in range(step_count + 1):
        rel_t = min(duration, step / rate_hz)
        source_t = frames[0].t + rel_t
        while source_index < len(frames) - 2 and frames[source_index + 1].t < source_t:
            source_index += 1
        left = frames[source_index]
        right = frames[min(source_index + 1, len(frames) - 1)]
        span = right.t - left.t
        ratio = 0.0 if span <= 0 else (source_t - left.t) / span
        values = [a + (b - a) * ratio for a, b in zip(left.values, right.values)]
        samples.append(VectorSample(round(rel_t, 10), values))
    return samples
