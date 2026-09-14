from __future__ import annotations

import math

import pytest

from dual_arm_protocol_tests.robot_model import DEFAULT_HOME_Q16, load_default_robot_model
from dual_arm_protocol_tests.trajectory import (
    VectorSample,
    interpolate_vectors,
    nearest_reference,
    normalize_quat,
    slerp_quat,
)
from dual_arm_protocol_tests.trajectory_factory import (
    build_servoj_target_trajectory,
    build_servop_target_trajectory,
    smoothstep5,
)


def test_interpolate_vectors_includes_start_and_target() -> None:
    samples = interpolate_vectors([0.0, 1.0], [1.0, 3.0], duration_sec=1.0, rate_hz=2.0)

    assert [sample.t for sample in samples] == [0.0, 0.5, 1.0]
    assert samples[0].values == [0.0, 1.0]
    assert samples[-1].values == [1.0, 3.0]


def test_nearest_reference_prefers_earlier_sample_on_tie() -> None:
    samples = [VectorSample(0.0, [0.0]), VectorSample(1.0, [1.0])]

    assert nearest_reference(samples, 0.5) == samples[0]


def test_quaternion_helpers_reject_zero_and_return_unit_quaternion() -> None:
    with pytest.raises(ValueError, match="must not be zero"):
        normalize_quat([0.0, 0.0, 0.0, 0.0])

    result = slerp_quat([1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], 0.5)

    assert math.isclose(sum(value * value for value in result), 1.0, abs_tol=1e-12)


def test_smoothstep5_clamps_ratio_to_unit_interval() -> None:
    assert smoothstep5(-1.0) == 0.0
    assert smoothstep5(2.0) == 1.0


def test_build_servoj_target_trajectory_preserves_endpoints() -> None:
    model = load_default_robot_model()
    target = list(DEFAULT_HOME_Q16)
    target[0] += 0.05

    trajectory = build_servoj_target_trajectory(
        list(DEFAULT_HOME_Q16),
        target,
        model=model,
        duration_sec=1.0,
        rate_hz=4.0,
    )

    assert trajectory.planned_duration_sec == 1.0
    assert trajectory.samples[0].values == DEFAULT_HOME_Q16
    assert trajectory.samples[-1].values == target
    assert trajectory.completed_segments == 1


def test_build_servop_target_trajectory_rejects_non_unit_quaternions() -> None:
    invalid_pose = [0.5, 0.28, -0.25, 2.0, 0.0, 0.0, 0.0] * 2

    with pytest.raises(ValueError, match="quaternions must be normalized"):
        build_servop_target_trajectory(
            invalid_pose,
            invalid_pose,
            duration_sec=1.0,
            rate_hz=4.0,
        )
