from __future__ import annotations

import pytest

from dual_arm_protocol_tests.case_factory import arrival_times, expand_generator


def test_arrival_times_include_bounds_and_strictly_increase() -> None:
    values = arrival_times(count=4, minimum=1.0, maximum=2.5)

    assert values[0] == 1.0
    assert values[-1] == 2.5
    assert all(left < right for left, right in zip(values, values[1:]))


def test_arrival_times_reject_invalid_range() -> None:
    with pytest.raises(ValueError, match="invalid arrival time range"):
        arrival_times(count=1, minimum=1.0, maximum=2.0)


def test_expand_move_generator_uses_group_waypoints_and_duration_field() -> None:
    cases = expand_generator(
        {
            "type": "moveJ",
            "id_prefix": "moveJ_ci",
            "count": 2,
            "time_range_sec": [1.0, 2.0],
            "simulation_only": True,
            "groups": [
                {"waypoints": [[0.0] * 14]},
                {"waypoints": [[0.1] * 14]},
            ],
        }
    )

    assert [case["id"] for case in cases] == ["moveJ_ci_01", "moveJ_ci_02"]
    assert [case["duration"] for case in cases] == [1.0, 2.0]
    assert cases[0]["simulation_only"] is True
    assert cases[1]["waypoints"] == [[0.1] * 14]


def test_expand_servo_generator_uses_segment_duration_field() -> None:
    cases = expand_generator(
        {
            "type": "servoJ",
            "count": 2,
            "time_range_sec": [1.0, 2.0],
            "pattern_count": 1,
        }
    )

    assert [case["segment_duration"] for case in cases] == [1.0, 2.0]
    assert all("duration" not in case for case in cases)


def test_expand_vla_generator_preserves_timestamps() -> None:
    cases = expand_generator(
        {
            "type": "vlaServoJ",
            "id_prefix": "vla_ci",
            "count": 2,
            "states_file": "examples/vla_height/states_controller_safe.txt",
        }
    )

    assert [case["id"] for case in cases] == ["vla_ci_01", "vla_ci_02"]
    assert all(case["preserve_timestamps"] is True for case in cases)
