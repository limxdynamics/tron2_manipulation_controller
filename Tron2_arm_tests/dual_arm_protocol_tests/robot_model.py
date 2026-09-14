from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
import math
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL_PATH = (
    PROJECT_ROOT
    / "robot_description/tron2a/DACH_TRON2A/xml/robot_grasper.xml"
)

PROTOCOL_JOINT_NAMES = [
    "proximal_pitch_L_Joint",
    "proximal_roll_L_Joint",
    "proximal_yaw_L_Joint",
    "elbow_L_Joint",
    "wrist_yaw_L_Joint",
    "wrist_pitch_L_Joint",
    "wrist_roll_L_Joint",
    "proximal_pitch_R_Joint",
    "proximal_roll_R_Joint",
    "proximal_yaw_R_Joint",
    "elbow_R_Joint",
    "wrist_yaw_R_Joint",
    "wrist_pitch_R_Joint",
    "wrist_roll_R_Joint",
    "head_pitch_Joint",
    "head_yaw_Joint",
]

DEFAULT_HOME_Q16 = [
    0.0,
    0.25,
    0.0,
    -1.57,
    0.236,
    0.0,
    0.0,
    0.0,
    -0.25,
    0.0,
    -1.57,
    -0.236,
    0.0,
    0.0,
    0.0,
    0.0,
]

DEFAULT_GROUP_AMPLITUDES = {
    "proximal_pitch": 0.30,
    "proximal_roll": 0.20,
    "proximal_yaw": 0.35,
    "elbow": 0.35,
    "wrist_yaw": 0.25,
    "wrist_pitch": 0.25,
    "wrist_roll": 0.30,
    "head_pitch": 0.25,
    "head_yaw": 0.35,
}


@dataclass(frozen=True)
class JointLimit:
    name: str
    lower: float
    upper: float

    def contains(self, value: float, *, margin: float = 0.0) -> bool:
        return self.lower + margin <= value <= self.upper - margin


CONTROLLER_ARM_LIMITS = tuple(
    JointLimit(name, lower, upper)
    for name, lower, upper in zip(
        PROTOCOL_JOINT_NAMES[:14],
        (
            -3.14159,
            -0.261799,
            -3.66519,
            -2.61799,
            -1.74533,
            -0.7505,
            -0.7505,
            -3.14159,
            -2.96706,
            -1.48353,
            -2.61799,
            -1.39626,
            -0.7505,
            -0.7505,
        ),
        (
            2.60054,
            2.96706,
            1.48353,
            0.523599,
            1.39626,
            0.7505,
            0.7505,
            2.60054,
            0.261799,
            3.66519,
            0.523599,
            1.74533,
            0.7505,
            0.7505,
        ),
    )
)


@dataclass(frozen=True)
class RobotModel:
    joints: Tuple[JointLimit, ...]

    @property
    def limits(self) -> Tuple[JointLimit, ...]:
        return self.joints

    @property
    def arm_limits(self) -> Tuple[JointLimit, ...]:
        return self.joints[:14]

    @property
    def head_limits(self) -> Tuple[JointLimit, ...]:
        return self.joints[14:16]

    @property
    def controller_limits(self) -> Tuple[JointLimit, ...]:
        return CONTROLLER_ARM_LIMITS

    @property
    def effective_limits(self) -> Tuple[JointLimit, ...]:
        arm = tuple(
            JointLimit(
                xml.name,
                max(xml.lower, controller.lower),
                min(xml.upper, controller.upper),
            )
            for xml, controller in zip(
                self.arm_limits,
                self.controller_limits,
            )
        )
        return arm + self.head_limits

    def safe_limits(
        self,
        *,
        home: Optional[List[float]] = None,
        edge_margin: float = 0.05,
        amplitudes: Optional[Dict[str, float]] = None,
    ) -> List[JointLimit]:
        center = home or DEFAULT_HOME_Q16
        group_amplitudes = dict(DEFAULT_GROUP_AMPLITUDES)
        group_amplitudes.update(amplitudes or {})
        safe: List[JointLimit] = []
        for idx, limit in enumerate(self.effective_limits):
            amp = _amplitude_for_joint(limit.name, group_amplitudes)
            lower = max(limit.lower + edge_margin, center[idx] - amp)
            upper = min(limit.upper - edge_margin, center[idx] + amp)
            if lower > upper:
                midpoint = (limit.lower + limit.upper) / 2.0
                lower = upper = midpoint
            safe.append(JointLimit(limit.name, lower, upper))
        return safe

    def validate_hard(self, values: Iterable[float], *, margin: float = 0.0) -> List[str]:
        return _validate_against_limits(list(values), self.joints, margin=margin)

    def validate_controller(
        self,
        values: Iterable[float],
        *,
        margin: float = 0.0,
    ) -> List[str]:
        supplied = list(values)
        if len(supplied) == len(self.joints):
            supplied = supplied[:14]
        return _validate_against_limits(
            supplied,
            self.controller_limits,
            margin=margin,
        )

    def validate_effective(
        self,
        values: Iterable[float],
        *,
        margin: float = 0.0,
    ) -> List[str]:
        return _validate_against_limits(
            list(values),
            self.effective_limits,
            margin=margin,
        )

    def validate_safe(self, values: Iterable[float], *, margin: float = 0.0) -> List[str]:
        return _validate_against_limits(list(values), self.safe_limits(), margin=margin)


@lru_cache(maxsize=1)
def load_default_robot_model() -> RobotModel:
    return load_robot_model(DEFAULT_MODEL_PATH)


def load_robot_model(path: Path) -> RobotModel:
    root = ET.parse(path).getroot()
    by_name: Dict[str, JointLimit] = {}
    for element in root.iter("joint"):
        name = element.get("name")
        range_text = element.get("range")
        if not name or not range_text:
            continue
        parts = [float(part) for part in range_text.split()]
        if len(parts) != 2:
            continue
        by_name[name] = JointLimit(name, parts[0], parts[1])

    missing = [name for name in PROTOCOL_JOINT_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"model missing protocol joints: {missing}")
    return RobotModel(
        tuple(by_name[name] for name in PROTOCOL_JOINT_NAMES)
    )


def _validate_against_limits(
    values: List[float],
    limits: Sequence[JointLimit],
    *,
    margin: float,
) -> List[str]:
    if len(values) != len(limits):
        return [f"expected {len(limits)} values, got {len(values)}"]
    errors: List[str] = []
    for idx, (value, limit) in enumerate(zip(values, limits)):
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            errors.append(f"joint[{idx}] is not finite")
        elif not limit.contains(float(value), margin=margin):
            errors.append(
                f"joint[{idx}]={value} outside {limit.name} "
                f"[{limit.lower + margin}, {limit.upper - margin}]"
            )
    return errors


def _amplitude_for_joint(name: str, amplitudes: Dict[str, float]) -> float:
    base = name.replace("_Joint", "")
    if base.startswith("head_pitch"):
        return amplitudes["head_pitch"]
    if base.startswith("head_yaw"):
        return amplitudes["head_yaw"]
    for key, value in amplitudes.items():
        if base.startswith(key):
            return value
    return 0.2
