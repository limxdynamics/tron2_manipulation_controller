from __future__ import annotations

import pytest

from dual_arm_protocol_tests.playback import (
    generate_preroll,
    load_vla_states_with_stats,
    resample_frames,
)
from dual_arm_protocol_tests.robot_model import DEFAULT_HOME_Q16, load_default_robot_model
from dual_arm_protocol_tests.trajectory import VectorSample


HEADER = (
    "episode step time abad_L hip_L yaw_L knee_L wrist_yaw_L "
    "wrist_pitch_L wrist_roll_L unused abad_R hip_R yaw_R knee_R "
    "wrist_yaw_R wrist_pitch_R wrist_roll_R unused"
)


def _row(timestamp: float, q16: list[float]) -> str:
    columns = [
        0,
        0,
        timestamp,
        *q16[0:7],
        0,
        *q16[7:14],
        0,
    ]
    return " ".join(str(value) for value in columns)


def test_load_vla_states_with_stats_accepts_valid_states_file(tmp_path) -> None:
    q0 = list(DEFAULT_HOME_Q16)
    q1 = list(DEFAULT_HOME_Q16)
    q1[0] += 0.01
    path = tmp_path / "states.txt"
    path.write_text(
        "\n".join([HEADER, _row(0.0, q0), _row(0.1, q1)]),
        encoding="utf-8",
    )

    result = load_vla_states_with_stats(
        path,
        list(DEFAULT_HOME_Q16),
        model=load_default_robot_model(),
    )

    assert len(result.frames) == 2
    assert result.frames[0].t == 0.0
    assert result.frames[1].values[0] == q1[0]
    assert result.stats.as_dict() == {
        "clamped_value_count": 0,
        "clamped_frame_count": 0,
        "max_clamp_correction_rad": 0.0,
    }


def test_load_vla_states_with_stats_rejects_non_monotonic_timestamps(tmp_path) -> None:
    path = tmp_path / "states.txt"
    path.write_text(
        "\n".join(
            [
                HEADER,
                _row(0.1, list(DEFAULT_HOME_Q16)),
                _row(0.1, list(DEFAULT_HOME_Q16)),
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="timestamp must be monotonic"):
        load_vla_states_with_stats(
            path,
            list(DEFAULT_HOME_Q16),
            model=load_default_robot_model(),
        )


def test_generate_preroll_returns_single_sample_when_already_at_target() -> None:
    samples = generate_preroll(
        list(DEFAULT_HOME_Q16),
        list(DEFAULT_HOME_Q16),
        rate_hz=300.0,
    )

    assert samples == [VectorSample(0.0, list(DEFAULT_HOME_Q16))]


def test_resample_frames_interpolates_to_zero_based_timeline() -> None:
    samples = resample_frames(
        [
            VectorSample(2.0, [0.0]),
            VectorSample(3.0, [1.0]),
        ],
        rate_hz=2.0,
    )

    assert [sample.t for sample in samples] == [0.0, 0.5, 1.0]
    assert [sample.values for sample in samples] == [[0.0], [0.5], [1.0]]
