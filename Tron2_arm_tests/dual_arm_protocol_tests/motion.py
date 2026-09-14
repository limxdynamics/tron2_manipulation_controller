from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable, List, Optional
import math
import time

from .metrics import joint_error, pose_error
from .state import JointState, MovePose


@dataclass(frozen=True)
class WaitResult:
    ok: bool
    code: str
    max_error: float
    samples: int
    state: Optional[Any] = None
    elapsed_sec: float = 0.0


def wait_for_joint_target(
    client: Any,
    target: List[float],
    *,
    indices: Iterable[int],
    tolerance_rad: float,
    velocity_rad_s: float,
    timeout_sec: float,
    poll_sec: float,
    stable_samples: int = 1,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> WaitResult:
    selected = list(indices)
    start = clock()
    deadline = start + max(0.0, timeout_sec)
    samples = 0
    stable_count = 0
    last_error = float("inf")
    last_state: Optional[JointState] = None

    while True:
        state = client.get_joint_state()
        samples += 1
        last_state = state
        try:
            actual = [state.q[idx] for idx in selected]
            desired = [target[idx] for idx in selected]
            velocities = [state.dq[idx] for idx in selected] if state.dq else [0.0 for _ in selected]
        except (IndexError, TypeError):
            return WaitResult(False, "invalid_joint_state", float("inf"), samples, state, clock() - start)

        if not all(math.isfinite(value) for value in actual + desired + velocities):
            return WaitResult(False, "non_finite_joint_state", float("inf"), samples, state, clock() - start)

        error = joint_error(desired, actual)
        last_error = error.max_abs
        velocity_ok = max(abs(value) for value in velocities) <= velocity_rad_s if velocities else True
        if error.max_abs <= tolerance_rad and velocity_ok:
            stable_count += 1
            if stable_count >= stable_samples:
                return WaitResult(True, "arrived", error.max_abs, samples, state, clock() - start)
        else:
            stable_count = 0

        if clock() >= deadline:
            break
        if poll_sec > 0:
            sleep(min(poll_sec, max(0.0, deadline - clock())))

    return WaitResult(False, "timeout", last_error, samples, last_state, clock() - start)


def wait_for_pose_target(
    client: Any,
    target_pose: List[float],
    *,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
    timeout_sec: float,
    poll_sec: float,
    stable_samples: int = 1,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> WaitResult:
    start = clock()
    deadline = start + max(0.0, timeout_sec)
    samples = 0
    stable_count = 0
    max_error = float("inf")
    last_pose: Optional[MovePose] = None

    while True:
        pose = client.get_move_pose()
        samples += 1
        last_pose = pose
        actual = pose.as_flat_pose()
        if len(actual) != 14 or len(target_pose) != 14:
            return WaitResult(False, "invalid_pose_state", float("inf"), samples, pose, clock() - start)
        error = pose_error(target_pose, actual)
        max_error = max(error.max_position, error.max_orientation_rad)
        if (
            error.max_position <= position_tolerance_m
            and error.max_orientation_rad <= orientation_tolerance_rad
        ):
            stable_count += 1
            if stable_count >= stable_samples:
                return WaitResult(True, "arrived", max_error, samples, pose, clock() - start)
        else:
            stable_count = 0
        if clock() >= deadline:
            break
        if poll_sec > 0:
            sleep(min(poll_sec, max(0.0, deadline - clock())))

    return WaitResult(False, "timeout", max_error, samples, last_pose, clock() - start)
