from __future__ import annotations

import pytest

from dual_arm_protocol_tests.protocol import (
    build_request,
    expected_response_title,
    validate_command_payload,
)
from dual_arm_protocol_tests.robot_model import DEFAULT_HOME_Q16
from dual_arm_protocol_tests.trajectory_factory import DEFAULT_REFERENCE_POSE


def test_build_request_uses_supplied_identity_and_guid() -> None:
    request = build_request(
        "request_movej",
        {"joint": [0.0] * 14, "time": 1.0},
        accid="DACH_TRON2A_001",
        guid="fixed-guid",
        timestamp_ms=123,
    )

    assert request == {
        "accid": "DACH_TRON2A_001",
        "title": "request_movej",
        "timestamp": 123,
        "guid": "fixed-guid",
        "data": {"joint": [0.0] * 14, "time": 1.0},
    }


def test_expected_response_title_requires_request_prefix() -> None:
    assert expected_response_title("request_servoj") == "response_servoj"

    with pytest.raises(ValueError, match="request title must start"):
        expected_response_title("notify_servoJ")


def test_validate_movej_payload_accepts_controller_home_position() -> None:
    errors = validate_command_payload(
        "moveJ",
        {"time": 1.0, "joint": DEFAULT_HOME_Q16[:14]},
    )

    assert errors == []


def test_validate_servoj_payload_checks_filter_ratio_and_head_joints() -> None:
    valid = validate_command_payload(
        "servoJ",
        {"q": DEFAULT_HOME_Q16, "filter_ratio": 1.0},
    )
    invalid = validate_command_payload(
        "servoJ",
        {"q": DEFAULT_HOME_Q16[:14] + [999.0, 0.0], "filter_ratio": 1.5},
    )

    assert valid == []
    assert any("filter_ratio" in error for error in invalid)
    assert any("head_pitch" in error for error in invalid)


def test_validate_movep_payload_accepts_reference_pose() -> None:
    assert validate_command_payload("moveP", {"time": 1.0, "pos": list(DEFAULT_REFERENCE_POSE)}) == []


def test_validate_servop_q_payload_requires_positive_period() -> None:
    errors = validate_command_payload(
        "servoP",
        {"q": list(DEFAULT_REFERENCE_POSE), "t": 0.0},
    )

    assert errors == ["t must be a positive finite number"]
