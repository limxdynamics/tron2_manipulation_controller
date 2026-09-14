from __future__ import annotations

from pathlib import Path

from dual_arm_protocol_tests.robot_model import (
    DEFAULT_MODEL_PATH,
    DEFAULT_HOME_Q16,
    PROTOCOL_JOINT_NAMES,
    load_default_robot_model,
)


def test_default_model_path_uses_top_level_robot_description() -> None:
    expected = (
        Path(__file__).resolve().parents[2]
        / "robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml"
    )

    assert DEFAULT_MODEL_PATH == expected


def test_default_robot_model_loads_protocol_joint_limits() -> None:
    model = load_default_robot_model()

    assert [limit.name for limit in model.limits] == PROTOCOL_JOINT_NAMES
    assert len(model.arm_limits) == 14
    assert len(model.head_limits) == 2


def test_default_home_is_inside_effective_and_safe_limits() -> None:
    model = load_default_robot_model()

    assert model.validate_effective(DEFAULT_HOME_Q16) == []
    assert model.validate_safe(DEFAULT_HOME_Q16) == []


def test_validation_reports_wrong_joint_count() -> None:
    model = load_default_robot_model()

    assert model.validate_hard([0.0]) == ["expected 16 values, got 1"]
