from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class JointState:
    q: List[float]
    dq: List[float] = field(default_factory=list)
    names: List[str] = field(default_factory=list)
    tau: List[float] = field(default_factory=list)
    timestamp_ms: Optional[int] = None

    @classmethod
    def from_response(cls, response: Dict[str, Any]) -> "JointState":
        data = response.get("data", response)
        return cls(
            q=[float(value) for value in data.get("q", [])],
            dq=[float(value) for value in data.get("dq", data.get("v", []))],
            names=list(data.get("names", [])),
            tau=[float(value) for value in data.get("tau", [])],
            timestamp_ms=data.get("timestamp"),
        )

    @property
    def arm_q(self) -> List[float]:
        return self.q[:14]

    @property
    def head_q(self) -> List[float]:
        return self.q[14:16]


@dataclass
class MovePose:
    left_position: List[float]
    left_quat: List[float]
    right_position: List[float]
    right_quat: List[float]
    timestamp_ms: Optional[int] = None

    @classmethod
    def from_response(cls, response: Dict[str, Any]) -> "MovePose":
        data = response.get("data", response)
        return cls(
            left_position=[float(value) for value in data.get("left_position", [])],
            left_quat=[float(value) for value in data.get("left_quat", [])],
            right_position=[float(value) for value in data.get("right_position", [])],
            right_quat=[float(value) for value in data.get("right_quat", [])],
            timestamp_ms=data.get("timestamp"),
        )

    def as_flat_pose(self) -> List[float]:
        return self.left_position + self.left_quat + self.right_position + self.right_quat


@dataclass
class StateCache:
    joint_state: Optional[JointState] = None
    move_pose: Optional[MovePose] = None

    def update_joint_state(self, state: JointState) -> JointState:
        self.joint_state = state
        return state

    def update_move_pose(self, pose: MovePose) -> MovePose:
        self.move_pose = pose
        return pose

    def snapshot(self) -> Dict[str, Any]:
        return {
            "joint_q": list(self.joint_state.q) if self.joint_state else [],
            "pose": self.move_pose.as_flat_pose() if self.move_pose else [],
        }
