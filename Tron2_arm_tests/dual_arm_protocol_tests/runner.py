from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional

from .metrics import joint_error, pose_error
from .motion import WaitResult, wait_for_joint_target, wait_for_pose_target
from .playback import (
    generate_preroll,
    load_vla_states_with_stats,
    resample_frames,
)
from .protocol import validate_command_payload
from .robot_model import DEFAULT_HOME_Q16, load_default_robot_model
from .state import JointState, MovePose
from .trajectory import VectorSample, interpolate_pose, interpolate_vectors
from .trajectory_factory import (
    build_servoj_loop_trajectory,
    build_servoj_target_trajectory,
    build_servop_loop_trajectory,
    build_servop_target_trajectory,
    resolve_case_waypoints,
)


DEFAULT_THRESHOLDS = {
    "joint_max_abs_rad": 0.05,
    "head_max_abs_rad": 0.05,
    "pose_position_m": 0.03,
    "pose_orientation_rad": 0.15,
}

_SEGMENT_BOUNDARY_EPSILON_SEC = 1e-9


@dataclass
class CaseResult:
    case_id: str
    case_type: str
    ok: bool
    result: str
    metrics: Dict[str, float] = field(default_factory=dict)
    start_state: Dict[str, Any] = field(default_factory=dict)
    final_state: Dict[str, Any] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)
    response_status: str = ""
    failure_stage: Optional[str] = None
    case_metadata: Dict[str, Any] = field(default_factory=dict)

    def as_row(self) -> Dict[str, Any]:
        core = {
            "id": self.case_id,
            "type": self.case_type,
            "ok": self.ok,
            "result": self.result,
            "messages": "; ".join(self.messages),
            "response_status": self.response_status,
            "failure_stage": self.failure_stage or "",
            "start_state": json.dumps(self.start_state, ensure_ascii=False),
            "final_state": json.dumps(self.final_state, ensure_ascii=False),
        }
        row = dict(self.metrics)
        row.update(self.case_metadata)
        row.update(core)
        return row

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.case_id,
            "type": self.case_type,
            "ok": self.ok,
            "result": self.result,
            "response_status": self.response_status,
            "failure_stage": self.failure_stage,
            "case_metadata": dict(self.case_metadata),
            "metrics": dict(self.metrics),
            "start_state": dict(self.start_state),
            "final_state": dict(self.final_state),
            "messages": list(self.messages),
        }


@dataclass
class LoopStats:
    active_elapsed_sec: float = 0.0
    completed_segments: int = 0
    completed_cycles: int = 0
    failure_stage: Optional[str] = None


class DualArmTestRunner:
    def __init__(
        self,
        client: Any,
        config: Optional[Dict[str, Any]] = None,
        *,
        monotonic: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.client = client
        self.config = config or {}
        self._monotonic = monotonic
        self._sleep = sleep
        self.thresholds = dict(DEFAULT_THRESHOLDS)
        self.thresholds.update(self.config.get("thresholds", {}))
        self.stop_on_failure = self.config.get("safety", {}).get("stop_on_failure", True)

    def run_segment_loop(
        self,
        *,
        active_duration_sec: float,
        segments: Iterable[Any],
        execute_segment: Callable[[Any], bool],
    ) -> LoopStats:
        """Repeat protocol-agnostic segments until the active duration is met."""
        duration = float(active_duration_sec)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("active_duration_sec must be a finite positive number")

        cycle_segments = list(segments)
        if not cycle_segments:
            raise ValueError("segments must not be empty")

        stats = LoopStats()
        start = self._monotonic()
        cycle_start = start
        while True:
            for index, segment in enumerate(cycle_segments):
                succeeded = execute_segment(segment)
                now = self._monotonic()
                stats.active_elapsed_sec = max(0.0, now - start)
                if not succeeded:
                    stats.failure_stage = _segment_stage(segment, index)
                    return stats

                stats.completed_segments += 1
                if index == len(cycle_segments) - 1:
                    stats.completed_cycles += 1
                    if now <= cycle_start and stats.active_elapsed_sec < duration:
                        raise RuntimeError("segment callback did not advance the monotonic clock")
                    cycle_start = now

                if stats.completed_cycles >= 1 and stats.active_elapsed_sec >= duration:
                    return stats

    def run_all(
        self,
        cases: Iterable[Dict[str, Any]],
        on_result: Optional[Callable[[CaseResult], None]] = None,
    ) -> List[CaseResult]:
        results: List[CaseResult] = []
        for case in cases:
            try:
                result = self.run_case(case)
            except Exception as exc:
                result = _runtime_exception_result(case, exc)
            results.append(result)
            if on_result is not None:
                on_result(result)
            if self.stop_on_failure and not result.ok:
                break
        return results

    def run_case(self, case: Dict[str, Any]) -> CaseResult:
        case_type = str(case["type"])
        if case_type == "moveJ":
            result = self._run_movej(case)
        elif case_type == "moveP":
            result = self._run_movep(case)
        elif case_type == "moveH":
            result = self._run_moveh(case)
        elif case_type == "servoJ":
            result = self._run_servoj(case)
        elif case_type == "servoP":
            result = self._run_servop(case)
        elif case_type == "vlaServoJ":
            result = self._run_vla_servoj(case)
        else:
            result = CaseResult(
                case["id"],
                case_type,
                False,
                "unsupported_case",
            )
        result.case_metadata = _case_report_metadata(case)
        return result

    def _run_movej(self, case: Dict[str, Any]) -> CaseResult:
        if "active_duration_sec" in case:
            return self._run_long_move(case, "moveJ")
        state = self.client.get_joint_state()
        target = _float_list(case["target_q"])
        payload = {"time": float(case["duration"]), "joint": target}
        validation = validate_command_payload("moveJ", payload)
        if validation:
            return _invalid_result(case, validation, {"joint_q": state.q})
        response = self.client.request("request_movej", payload)
        if not _response_ok(response):
            return _response_fail_result(case, "moveJ", response, {"joint_q": state.q})
        wait = self._wait_joint(target + state.head_q, range(14), case)
        final_state = wait.state if isinstance(wait.state, JointState) else self.client.get_joint_state()
        error = joint_error(target, final_state.arm_q)
        ok = wait.ok and error.max_abs <= self.thresholds["joint_max_abs_rad"]
        result = "ok" if ok else wait.code
        return CaseResult(
            case["id"],
            "moveJ",
            ok,
            result,
            metrics={
                "joint_max_abs_rad": error.max_abs,
                "joint_rms_rad": error.rms,
                "arrive_elapsed_sec": wait.elapsed_sec,
                "arrive_samples": float(wait.samples),
            },
            start_state={"joint_q": state.q},
            final_state={"joint_q": final_state.q},
            response_status=_response_status(response),
        )

    def _run_movep(self, case: Dict[str, Any]) -> CaseResult:
        if "active_duration_sec" in case:
            return self._run_long_move(case, "moveP")
        pose = self.client.get_move_pose()
        target = _float_list(case["target_pose"])
        payload = {"time": float(case["duration"]), "pos": target}
        validation = validate_command_payload("moveP", payload)
        if validation:
            return _invalid_result(case, validation, {"pose": pose.as_flat_pose()})
        response = self.client.request("request_movep", payload)
        if not _response_ok(response):
            return _response_fail_result(case, "moveP", response, {"pose": pose.as_flat_pose()})
        wait = self._wait_pose(target, case)
        final_pose = wait.state if isinstance(wait.state, MovePose) else self.client.get_move_pose()
        error = pose_error(target, final_pose.as_flat_pose())
        ok = (
            wait.ok
            and error.max_position <= self.thresholds["pose_position_m"]
            and error.max_orientation_rad <= self.thresholds["pose_orientation_rad"]
        )
        result = "ok" if ok else wait.code
        return CaseResult(
            case["id"],
            "moveP",
            ok,
            result,
            metrics={
                "pose_position_m": error.max_position,
                "pose_orientation_rad": error.max_orientation_rad,
                "arrive_elapsed_sec": wait.elapsed_sec,
                "arrive_samples": float(wait.samples),
            },
            start_state={"pose": pose.as_flat_pose()},
            final_state={"pose": final_pose.as_flat_pose()},
            response_status=_response_status(response),
        )

    def _run_moveh(self, case: Dict[str, Any]) -> CaseResult:
        if "active_duration_sec" in case:
            return self._run_long_move(case, "moveH")
        state = self.client.get_joint_state()
        target = _float_list(case["target_head"])
        payload = {"time": float(case["duration"]), "joint": target}
        validation = validate_command_payload("moveH", payload)
        if validation:
            return _invalid_result(case, validation, {"joint_q": state.q})
        response = self.client.request("request_moveh", payload)
        if not _response_ok(response):
            return _response_fail_result(case, "moveH", response, {"joint_q": state.q})
        target_q = list(state.q[:14]) + target
        wait = self._wait_joint(target_q, range(14, 16), case)
        final_state = wait.state if isinstance(wait.state, JointState) else self.client.get_joint_state()
        error = joint_error(target, final_state.head_q)
        ok = wait.ok and error.max_abs <= self.thresholds["head_max_abs_rad"]
        result = "ok" if ok else wait.code
        return CaseResult(
            case["id"],
            "moveH",
            ok,
            result,
            metrics={
                "head_max_abs_rad": error.max_abs,
                "head_rms_rad": error.rms,
                "arrive_elapsed_sec": wait.elapsed_sec,
                "arrive_samples": float(wait.samples),
            },
            start_state={"joint_q": state.q},
            final_state={"joint_q": final_state.q},
            response_status=_response_status(response),
        )

    def _run_long_move(self, case: Dict[str, Any], case_type: str) -> CaseResult:
        duration = float(case["duration"])
        if case_type == "moveP":
            initial = self.client.get_move_pose()
            start_state = {"pose": initial.as_flat_pose()}
            return_target = initial.as_flat_pose()
            request_title = "request_movep"
            payload_key = "pos"
        else:
            initial = self.client.get_joint_state()
            start_state = {"joint_q": initial.q}
            return_target = (
                list(DEFAULT_HOME_Q16[:14])
                if case_type == "moveJ"
                else list(DEFAULT_HOME_Q16[14:16])
            )
            request_title = "request_movej" if case_type == "moveJ" else "request_moveh"
            payload_key = "joint"

        try:
            waypoints = self._resolve_long_waypoints(
                case,
                case_type,
                reference_pose=return_target if case_type == "moveP" else None,
            )
        except ValueError as exc:
            return _invalid_long_result(
                case,
                [str(exc)],
                start_state,
                failure_stage="pattern_resolution",
            )

        validation_errors: List[str] = []
        for index, waypoint in enumerate(waypoints):
            errors = validate_command_payload(
                case_type,
                {"time": duration, payload_key: waypoint},
            )
            validation_errors.extend(f"waypoint_{index + 1}: {error}" for error in errors)
        return_errors = validate_command_payload(
            case_type,
            {"time": duration, payload_key: return_target},
        )
        validation_errors.extend(f"safe_return: {error}" for error in return_errors)
        if validation_errors:
            return _invalid_long_result(
                case,
                validation_errors,
                start_state,
                failure_stage="payload_validation",
            )

        segments = [
            {"stage": f"waypoint_{index + 1}", "target": waypoint}
            for index, waypoint in enumerate(waypoints)
        ]
        next_deadline = self._monotonic()
        failure_status = ""

        def execute(segment: Dict[str, Any]) -> bool:
            nonlocal next_deadline, failure_status
            response = self.client.request(
                request_title,
                {"time": duration, payload_key: segment["target"]},
            )
            if not _response_ok(response):
                failure_status = _response_status(response) or "empty_result"
                return False
            next_deadline += duration
            self._sleep_until_deadline(next_deadline)
            return True

        stats = self.run_segment_loop(
            active_duration_sec=float(case["active_duration_sec"]),
            segments=segments,
            execute_segment=execute,
        )
        metrics = _loop_metrics(duration, stats)
        if stats.failure_stage:
            return CaseResult(
                case["id"],
                case_type,
                False,
                "response_fail",
                metrics=metrics,
                start_state=start_state,
                messages=[f"{stats.failure_stage}:{failure_status}"],
                response_status=failure_status,
                failure_stage=stats.failure_stage,
            )

        response = self.client.request(
            request_title,
            {"time": duration, payload_key: return_target},
        )
        if not _response_ok(response):
            status = _response_status(response) or "empty_result"
            return CaseResult(
                case["id"],
                case_type,
                False,
                "response_fail",
                metrics=metrics,
                start_state=start_state,
                messages=[f"safe_return:{status}"],
                response_status=status,
                failure_stage="safe_return",
            )

        if case_type == "moveJ":
            wait = self._wait_joint(return_target + initial.head_q, range(14), case)
            final = wait.state if isinstance(wait.state, JointState) else self.client.get_joint_state()
            error = joint_error(return_target, final.arm_q)
            metrics.update(
                {
                    "joint_max_abs_rad": error.max_abs,
                    "joint_rms_rad": error.rms,
                    "arrive_elapsed_sec": wait.elapsed_sec,
                    "arrive_samples": float(wait.samples),
                }
            )
            ok = wait.ok and error.max_abs <= self.thresholds["joint_max_abs_rad"]
            final_state = {"joint_q": final.q}
        elif case_type == "moveH":
            wait = self._wait_joint(initial.arm_q + return_target, range(14, 16), case)
            final = wait.state if isinstance(wait.state, JointState) else self.client.get_joint_state()
            error = joint_error(return_target, final.head_q)
            metrics.update(
                {
                    "head_max_abs_rad": error.max_abs,
                    "head_rms_rad": error.rms,
                    "arrive_elapsed_sec": wait.elapsed_sec,
                    "arrive_samples": float(wait.samples),
                }
            )
            ok = wait.ok and error.max_abs <= self.thresholds["head_max_abs_rad"]
            final_state = {"joint_q": final.q}
        else:
            wait = self._wait_pose(return_target, case)
            final = wait.state if isinstance(wait.state, MovePose) else self.client.get_move_pose()
            error = pose_error(return_target, final.as_flat_pose())
            metrics.update(
                {
                    "pose_position_m": error.max_position,
                    "pose_orientation_rad": error.max_orientation_rad,
                    "arrive_elapsed_sec": wait.elapsed_sec,
                    "arrive_samples": float(wait.samples),
                }
            )
            ok = (
                wait.ok
                and error.max_position <= self.thresholds["pose_position_m"]
                and error.max_orientation_rad <= self.thresholds["pose_orientation_rad"]
            )
            final_state = {"pose": final.as_flat_pose()}
        return CaseResult(
            case["id"],
            case_type,
            ok,
            "ok" if ok else wait.code,
            metrics=metrics,
            start_state=start_state,
            final_state=final_state,
            response_status=_response_status(response),
            failure_stage=None if ok else "final_stability",
        )

    def _resolve_long_waypoints(
        self,
        case: Dict[str, Any],
        case_type: str,
        *,
        reference_pose: Optional[List[float]] = None,
    ) -> List[List[float]]:
        return resolve_case_waypoints(
            {**case, "type": case_type},
            reference_pose=reference_pose,
        )

    def _sleep_until_deadline(self, deadline: float) -> None:
        remaining = deadline - self._monotonic()
        if remaining > 0.0:
            self._sleep(remaining)

    def _run_servoj(self, case: Dict[str, Any]) -> CaseResult:
        if "active_duration_sec" in case or "segment_duration" in case:
            return self._run_long_servo(case, "servoJ")
        state = self.client.get_joint_state()
        target = _float_list(case["target_q"])
        head = _float_list(case.get("target_head", state.head_q or [0.0, 0.0]))
        q16 = target + head
        validation = validate_command_payload("servoJ", {"q": q16})
        if validation:
            return _invalid_result(case, validation, {"joint_q": state.q})
        start_head = state.head_q if len(state.head_q) == 2 else head
        start_q = state.arm_q + start_head
        planned_duration = self._servoj_duration(start_q, q16, case)
        samples = interpolate_vectors(
            start_q,
            q16,
            duration_sec=planned_duration,
            rate_hz=float(case.get("send_rate_hz", self._default_servo_rate())),
        )
        start_time = self._monotonic()
        notifications: List[Dict[str, Any]] = []
        send_count = 0
        for sample in samples:
            self._sleep_until_sample(start_time, sample.t)
            self.client.request(
                "request_servoj",
                {"filter_ratio": float(case.get("filter_ratio", 1.0)), "q": sample.values},
                expect_response=False,
            )
            send_count += 1
            notifications.extend(self.client.drain_notifications("notify_servoJ"))
            if notifications:
                break
        elapsed = max(0.0, self._monotonic() - start_time)
        final_state = self.client.get_joint_state()
        error = joint_error(q16, final_state.q[:16])
        ok = not notifications and error.max_abs <= self.thresholds["joint_max_abs_rad"]
        result = "ok" if ok else "notify_or_accuracy_fail"
        return CaseResult(
            case["id"],
            "servoJ",
            ok,
            result,
            metrics={
                "joint_max_abs_rad": error.max_abs,
                "joint_rms_rad": error.rms,
                "planned_duration_sec": planned_duration,
                "planned_send_count": float(len(samples)),
                "actual_send_count": float(send_count),
                "actual_send_rate_hz": _actual_send_rate(send_count, elapsed),
            },
            start_state={"joint_q": state.q},
            final_state={"joint_q": final_state.q},
            messages=[json.dumps(item, ensure_ascii=False) for item in notifications],
        )

    def _run_servop(self, case: Dict[str, Any]) -> CaseResult:
        if "active_duration_sec" in case or "segment_duration" in case:
            return self._run_long_servo(case, "servoP")
        pose = self.client.get_move_pose()
        target = _float_list(case["target_pose"])
        payload_mode = str(case.get("payload_mode", "pos"))
        if payload_mode not in {"pos", "q"}:
            return _invalid_result(
                case,
                [f"unknown payload_mode: {payload_mode}"],
                {"pose": pose.as_flat_pose()},
            )
        rate_hz = float(case.get("send_rate_hz", self._default_servo_rate()))
        validation_payload = (
            {"q": target, "t": 1.0 / rate_hz}
            if payload_mode == "q"
            else {"pos": target}
        )
        validation = validate_command_payload("servoP", validation_payload)
        if validation:
            return _invalid_result(case, validation, {"pose": pose.as_flat_pose()})
        samples = interpolate_pose(
            pose.as_flat_pose(),
            target,
            duration_sec=float(case["duration"]),
            rate_hz=rate_hz,
        )
        start_time = self._monotonic()
        notifications: List[Dict[str, Any]] = []
        send_count = 0
        for sample in samples:
            self._sleep_until_sample(start_time, sample.t)
            payload = {"pos": sample.values}
            if payload_mode == "q":
                payload = {"q": sample.values, "t": 1.0 / rate_hz}
            self.client.request("request_servop", payload, expect_response=False)
            send_count += 1
            notifications.extend(self.client.drain_notifications("notify_servop"))
            if notifications:
                break
        elapsed = max(0.0, self._monotonic() - start_time)
        final_pose = self.client.get_move_pose()
        error = pose_error(target, final_pose.as_flat_pose())
        ok = (
            not notifications
            and error.max_position <= self.thresholds["pose_position_m"]
            and error.max_orientation_rad <= self.thresholds["pose_orientation_rad"]
        )
        result = "ok" if ok else "notify_or_accuracy_fail"
        return CaseResult(
            case["id"],
            "servoP",
            ok,
            result,
            metrics={
                "pose_position_m": error.max_position,
                "pose_orientation_rad": error.max_orientation_rad,
                "planned_send_count": float(len(samples)),
                "actual_send_count": float(send_count),
                "actual_send_rate_hz": _actual_send_rate(send_count, elapsed),
            },
            start_state={"pose": pose.as_flat_pose()},
            final_state={"pose": final_pose.as_flat_pose()},
            messages=[json.dumps(item, ensure_ascii=False) for item in notifications],
        )

    def _run_long_servo(
        self,
        case: Dict[str, Any],
        case_type: str,
    ) -> CaseResult:
        rate_hz = float(case.get("send_rate_hz", self._default_servo_rate()))
        active_duration = (
            float(case["active_duration_sec"])
            if "active_duration_sec" in case
            else None
        )
        model = load_default_robot_model()

        if case_type == "servoJ":
            state = self.client.get_joint_state()
            start = list(state.q[:16])
            start_state = {"joint_q": state.q}
            return_target = list(DEFAULT_HOME_Q16)
        else:
            pose = self.client.get_move_pose()
            start = pose.as_flat_pose()
            start_state = {"pose": start}
            return_target = list(start)

        try:
            segment_duration = float(case["segment_duration"])
        except KeyError:
            return _invalid_long_servo_result(
                case,
                case_type,
                start_state,
                segment_duration=0.0,
                message="segment_duration is required",
                failure_stage="segment_duration_validation",
            )

        try:
            waypoints = self._resolve_long_servo_waypoints(
                case,
                case_type,
                reference=start,
            )
        except (TypeError, ValueError) as exc:
            return _invalid_long_servo_result(
                case,
                case_type,
                start_state,
                segment_duration=segment_duration,
                message=str(exc),
                failure_stage="pattern_resolution",
            )

        try:
            single_target = active_duration is None
            target = waypoints[0]
            if case_type == "servoJ":
                if single_target:
                    active_trajectory = build_servoj_target_trajectory(
                        start,
                        target,
                        model=model,
                        duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                    return_trajectory = build_servoj_target_trajectory(
                        active_trajectory.samples[-1].values,
                        return_target,
                        model=model,
                        duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                else:
                    active_trajectory = build_servoj_loop_trajectory(
                        start,
                        waypoints,
                        model=model,
                        active_duration_sec=active_duration,
                        segment_duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                    return_trajectory = build_servoj_loop_trajectory(
                        active_trajectory.samples[-1].values,
                        [return_target],
                        model=model,
                        active_duration_sec=segment_duration,
                        segment_duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
            else:
                if single_target:
                    active_trajectory = build_servop_target_trajectory(
                        start,
                        target,
                        duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                    return_trajectory = build_servop_target_trajectory(
                        active_trajectory.samples[-1].values,
                        return_target,
                        duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                else:
                    active_trajectory = build_servop_loop_trajectory(
                        start,
                        waypoints,
                        active_duration_sec=active_duration,
                        segment_duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
                    return_trajectory = build_servop_loop_trajectory(
                        active_trajectory.samples[-1].values,
                        [return_target],
                        active_duration_sec=segment_duration,
                        segment_duration_sec=segment_duration,
                        rate_hz=rate_hz,
                    )
        except (TypeError, ValueError) as exc:
            return _invalid_long_servo_result(
                case,
                case_type,
                start_state,
                segment_duration=segment_duration,
                message=str(exc),
                failure_stage="trajectory_planning",
            )

        payload_mode = str(case.get("payload_mode", "pos"))
        filter_ratio = float(case.get("filter_ratio", 1.0))
        if case_type == "servoJ":
            validation = validate_command_payload(
                "servoJ",
                {"q": active_trajectory.samples[0].values, "filter_ratio": filter_ratio},
            )
            request_title = "request_servoj"
            notify_title = "notify_servoJ"

            def make_payload(values: List[float]) -> Dict[str, Any]:
                return {"filter_ratio": filter_ratio, "q": values}
        else:
            request_title = "request_servop"
            notify_title = "notify_servop"

            def make_payload(values: List[float]) -> Dict[str, Any]:
                if payload_mode == "q":
                    return {"q": values, "t": 1.0 / rate_hz}
                return {"pos": values}

            if payload_mode not in {"pos", "q"}:
                validation = [f"unknown payload_mode: {payload_mode}"]
            else:
                validation_payload = (
                    {"q": active_trajectory.samples[0].values, "t": 1.0 / rate_hz}
                    if payload_mode == "q"
                    else {"pos": active_trajectory.samples[0].values}
                )
                validation = validate_command_payload("servoP", validation_payload)

        if validation:
            return CaseResult(
                case["id"],
                case_type,
                False,
                "validation_fail",
                metrics=_long_servo_metrics(
                    segment_duration=segment_duration,
                    planned_duration=active_trajectory.planned_duration_sec,
                    active_elapsed=0.0,
                    active_planned_send_count=len(active_trajectory.samples),
                    active_actual_send_count=0,
                    return_planned_send_count=max(
                        0, len(return_trajectory.samples) - 1
                    ),
                    return_actual_send_count=0,
                    actual_send_rate=0.0,
                    completed_segments=0,
                    completed_cycles=0,
                ),
                start_state=start_state,
                messages=validation,
                failure_stage="payload_validation",
            )

        overall_start = self._monotonic()
        active_start = overall_start
        notifications: List[Dict[str, Any]] = []
        active_send_count = 0
        return_send_count = 0
        last_active_t = 0.0
        for sample in active_trajectory.samples:
            self._sleep_until_sample(active_start, sample.t)
            self.client.request(
                request_title,
                make_payload(sample.values),
                expect_response=False,
            )
            active_send_count += 1
            last_active_t = sample.t
            notifications.extend(self.client.drain_notifications(notify_title))
            if notifications:
                break

        active_elapsed = max(0.0, self._monotonic() - active_start)
        failure_stage: Optional[str] = None
        if notifications:
            failure_stage = "active_stream"
        else:
            return_start = self._monotonic()
            for sample in return_trajectory.samples[1:]:
                self._sleep_until_sample(return_start, sample.t)
                self.client.request(
                    request_title,
                    make_payload(sample.values),
                    expect_response=False,
                )
                return_send_count += 1
                notifications.extend(self.client.drain_notifications(notify_title))
                if notifications:
                    failure_stage = "safe_return"
                    break

        actual_rate = _actual_send_rate(active_send_count, active_elapsed)
        if failure_stage == "active_stream":
            completed_segments = min(
                active_trajectory.completed_segments,
                int(
                    math.floor(
                        (last_active_t + _SEGMENT_BOUNDARY_EPSILON_SEC)
                        / segment_duration
                    )
                ),
            )
            completed_cycles = completed_segments // len(waypoints)
        else:
            completed_segments = active_trajectory.completed_segments
            completed_cycles = active_trajectory.completed_cycles
        metrics = _long_servo_metrics(
            segment_duration=segment_duration,
            planned_duration=active_trajectory.planned_duration_sec,
            active_elapsed=active_elapsed,
            active_planned_send_count=len(active_trajectory.samples),
            active_actual_send_count=active_send_count,
            return_planned_send_count=max(
                0, len(return_trajectory.samples) - 1
            ),
            return_actual_send_count=return_send_count,
            actual_send_rate=actual_rate,
            completed_segments=completed_segments,
            completed_cycles=completed_cycles,
        )
        wait: Optional[WaitResult] = None
        final: Optional[Any] = None
        failure_result = "notify_or_accuracy_fail"
        exception_messages: List[str] = []
        if failure_stage is None:
            try:
                if case_type == "servoJ":
                    wait = self._wait_joint(return_target, range(16), case)
                else:
                    wait = self._wait_pose(return_target, case)
                final = wait.state
                metrics.update(
                    {
                        "return_stability_samples": float(wait.samples),
                        "return_stability_elapsed_sec": float(wait.elapsed_sec),
                    }
                )
            except Exception as exc:
                failure_stage = "safe_return/stability"
                failure_result = "exception"
                exception_messages.append(str(exc))

            try:
                final_notifications = self.client.drain_notifications(
                    notify_title
                )
                notifications.extend(final_notifications)
                if final_notifications:
                    failure_stage = "safe_return/notification"
                    failure_result = "notify_or_accuracy_fail"
            except Exception as exc:
                failure_stage = "safe_return/notification"
                failure_result = "exception"
                exception_messages.append(str(exc))

            if wait is not None and not wait.ok and failure_stage is None:
                failure_stage = "safe_return/stability"
                failure_result = wait.code

        if final is None:
            try:
                final = (
                    self.client.get_joint_state()
                    if case_type == "servoJ"
                    else self.client.get_move_pose()
                )
            except Exception as exc:
                if failure_stage is None:
                    failure_stage = "safe_return/stability"
                    failure_result = "exception"
                exception_messages.append(str(exc))

        accurate = False
        if case_type == "servoJ" and isinstance(final, JointState):
            error = joint_error(return_target, final.q[:16])
            metrics.update(
                {
                    "joint_max_abs_rad": error.max_abs,
                    "joint_rms_rad": error.rms,
                }
            )
            accurate = error.max_abs <= self.thresholds["joint_max_abs_rad"]
            final_state = {"joint_q": final.q}
        elif case_type == "servoP" and isinstance(final, MovePose):
            error = pose_error(return_target, final.as_flat_pose())
            metrics.update(
                {
                    "pose_position_m": error.max_position,
                    "pose_orientation_rad": error.max_orientation_rad,
                }
            )
            accurate = (
                error.max_position <= self.thresholds["pose_position_m"]
                and error.max_orientation_rad
                <= self.thresholds["pose_orientation_rad"]
            )
            final_state = {"pose": final.as_flat_pose()}
        else:
            final_state = {}

        ok = failure_stage is None and wait is not None and wait.ok and accurate
        if not ok and failure_stage is None:
            failure_stage = "safe_return/stability"
            failure_result = wait.code if wait is not None else "state_unavailable"
        return CaseResult(
            case["id"],
            case_type,
            ok,
            "ok" if ok else failure_result,
            metrics=metrics,
            start_state=start_state,
            final_state=final_state,
            messages=[
                *[
                    json.dumps(item, ensure_ascii=False)
                    for item in notifications
                ],
                *exception_messages,
            ],
            failure_stage=failure_stage,
        )

    def _resolve_long_servo_waypoints(
        self,
        case: Dict[str, Any],
        case_type: str,
        *,
        reference: List[float],
    ) -> List[List[float]]:
        return resolve_case_waypoints(
            {**case, "type": case_type},
            reference_pose=reference,
        )

    def _run_vla_servoj(self, case: Dict[str, Any]) -> CaseResult:
        repeat_index = float(case.get("repeat_index", 0))
        metrics = _vla_metrics(repeat_index=repeat_index)
        try:
            state = self.client.get_joint_state()
        except Exception as exc:
            return CaseResult(
                case["id"],
                "vlaServoJ",
                False,
                "validation_fail",
                metrics=metrics,
                messages=[str(exc)],
                failure_stage="joint_state",
            )
        start_state = {"joint_q": state.q}
        if len(state.q) < 16:
            return CaseResult(
                case["id"],
                "vlaServoJ",
                False,
                "validation_fail",
                metrics=metrics,
                start_state=start_state,
                messages=["joint state q must contain at least 16 values"],
                failure_stage="joint_state",
            )
        if not all(math.isfinite(value) for value in state.q[:16]):
            return CaseResult(
                case["id"],
                "vlaServoJ",
                False,
                "validation_fail",
                metrics=metrics,
                start_state=start_state,
                messages=["joint state q contains non-finite values"],
                failure_stage="joint_state",
            )
        if case.get("preserve_timestamps", True) is False:
            return CaseResult(
                case["id"],
                "vlaServoJ",
                False,
                "validation_fail",
                metrics=metrics,
                start_state=start_state,
                messages=[
                    "preserve_timestamps=false is not supported for VLA playback"
                ],
                failure_stage="timeline_validation",
            )

        model = load_default_robot_model()
        try:
            path = Path(case["states_file"])
            rate_hz = float(
                case.get("send_rate_hz", self._default_servo_rate())
            )
            max_velocity = float(case.get("max_velocity_rad_s", 0.5))
            max_source_velocity = float(
                case.get("max_source_velocity_rad_s", 3.0)
            )
            loaded = load_vla_states_with_stats(
                path,
                state.q[:16],
                model=model,
                max_joint_step_rad=float(case.get("max_joint_step_rad", 0.25)),
                max_source_velocity_rad_s=max_source_velocity,
                hard_limit_tolerance_rad=float(
                    case.get("hard_limit_tolerance_rad", 0.0)
                ),
            )
            frames = loaded.frames
            metrics.update(
                {
                    key: float(value)
                    for key, value in loaded.stats.as_dict().items()
                }
            )
            metrics["frame_count"] = float(len(frames))
            metrics["source_duration_sec"] = float(
                frames[-1].t - frames[0].t
            )
            source_samples = resample_frames(frames, rate_hz=rate_hz)
            warmup_samples = _constant_vector_samples(
                list(state.q[:16]),
                duration_sec=float(case.get("warmup_duration_sec", 0.0)),
                rate_hz=rate_hz,
            )
            transition_samples = generate_preroll(
                list(state.q[:16]),
                source_samples[0].values,
                rate_hz=rate_hz,
                max_velocity_rad_s=max_velocity,
            )
            return_samples = generate_preroll(
                source_samples[-1].values,
                list(DEFAULT_HOME_Q16),
                rate_hz=rate_hz,
                max_velocity_rad_s=max_velocity,
            )
            metrics["transition_duration_sec"] = float(
                transition_samples[-1].t
            )
            metrics["warmup_duration_sec"] = float(case.get("warmup_duration_sec", 0.0))
            metrics["warmup_planned_send_count"] = float(
                len(warmup_samples)
            )
            metrics["transition_planned_send_count"] = float(
                len(transition_samples)
            )
            metrics["source_planned_send_count"] = float(
                max(0, len(source_samples) - 1)
            )
            metrics["return_planned_send_count"] = float(
                max(0, len(return_samples) - 1)
            )
            metrics["planned_send_count"] = (
                metrics["warmup_planned_send_count"]
                + metrics["transition_planned_send_count"]
                + metrics["source_planned_send_count"]
                + metrics["return_planned_send_count"]
            )
        except (OSError, TypeError, ValueError) as exc:
            return CaseResult(
                case["id"],
                "vlaServoJ",
                False,
                "validation_fail",
                metrics=metrics,
                start_state=start_state,
                messages=[str(exc)],
                failure_stage="playback_validation",
            )

        transition_duration = metrics["transition_duration_sec"]
        filter_ratio = float(case.get("filter_ratio", 1.0))
        overall_start = self._monotonic()
        notifications: List[Dict[str, Any]] = []
        send_count = 0
        warmup_send_count = 0
        transition_send_count = 0
        source_send_count = 0
        return_send_count = 0
        source_first_send_time: Optional[float] = None
        source_last_send_time: Optional[float] = None
        active_elapsed = 0.0
        failure_stage: Optional[str] = None
        failure_result = "notify_or_accuracy_fail"
        exception_messages: List[str] = []
        last_target = list(state.q[:16])

        def stream(samples: List[Any], base_time: float, stage: str) -> bool:
            nonlocal send_count, warmup_send_count, transition_send_count
            nonlocal source_send_count, return_send_count
            nonlocal source_first_send_time, source_last_send_time
            nonlocal failure_stage, failure_result, last_target
            for sample in samples:
                self._sleep_until_sample(base_time, sample.t)
                send_time = self._monotonic()
                self.client.request(
                    "request_servoj",
                    {"filter_ratio": filter_ratio, "q": sample.values},
                    expect_response=False,
                )
                send_count += 1
                if stage == "warmup":
                    warmup_send_count += 1
                elif stage == "transition":
                    transition_send_count += 1
                elif stage == "source":
                    source_send_count += 1
                    if source_first_send_time is None:
                        source_first_send_time = send_time
                    source_last_send_time = send_time
                else:
                    return_send_count += 1
                last_target = sample.values
                try:
                    notifications.extend(
                        self.client.drain_notifications("notify_servoJ")
                    )
                except Exception as exc:
                    failure_stage = stage
                    failure_result = "exception"
                    exception_messages.append(str(exc))
                    return False
                if notifications:
                    failure_stage = stage
                    return False
            return True

        warmup_duration = metrics["warmup_duration_sec"]
        if stream(warmup_samples, overall_start, "warmup"):
            transition_base = overall_start + warmup_duration
            if stream(transition_samples, transition_base, "transition"):
                source_start = self._monotonic()
                source_base = transition_base + transition_duration
                source_ok = stream(source_samples[1:], source_base, "source")
                active_elapsed = max(0.0, self._monotonic() - source_start)
                if source_ok:
                    return_start = self._monotonic()
                    stream(return_samples[1:], return_start, "safe_return")

        source_elapsed = 0.0
        if (
            source_first_send_time is not None
            and source_last_send_time is not None
        ):
            source_elapsed = max(
                0.0, source_last_send_time - source_first_send_time
            )

        wait: Optional[WaitResult] = None
        final_state: Optional[JointState] = None
        if failure_stage is None:
            try:
                wait = self._wait_joint(
                    list(DEFAULT_HOME_Q16), range(16), case
                )
                if isinstance(wait.state, JointState):
                    final_state = wait.state
                metrics["return_stability_samples"] = float(wait.samples)
                metrics["return_stability_elapsed_sec"] = float(
                    wait.elapsed_sec
                )
                if not wait.ok:
                    failure_stage = "stability"
                    failure_result = wait.code
            except Exception as exc:
                failure_stage = "stability"
                failure_result = "exception"
                exception_messages.append(str(exc))

            try:
                final_notifications = self.client.drain_notifications(
                    "notify_servoJ"
                )
                notifications.extend(final_notifications)
                if final_notifications:
                    failure_stage = "final_notification"
                    failure_result = "notify_or_accuracy_fail"
            except Exception as exc:
                failure_stage = "final_notification"
                failure_result = "exception"
                exception_messages.append(str(exc))

        if final_state is None and failure_result != "exception":
            try:
                final_state = self.client.get_joint_state()
            except Exception as exc:
                if failure_stage is None:
                    failure_stage = "stability"
                    failure_result = "exception"
                exception_messages.append(str(exc))

        error_target = (
            list(DEFAULT_HOME_Q16)
            if failure_stage in {None, "safe_return", "stability", "final_notification"}
            else last_target
        )
        accurate = False
        final_state_dict: Dict[str, Any] = {}
        if isinstance(final_state, JointState):
            error = joint_error(error_target, final_state.q[:16])
            metrics["joint_max_abs_rad"] = float(error.max_abs)
            metrics["joint_rms_rad"] = float(error.rms)
            accurate = (
                error.max_abs <= self.thresholds["joint_max_abs_rad"]
            )
            final_state_dict = {"joint_q": final_state.q}

        if failure_stage is None and not accurate:
            failure_stage = "stability"
            failure_result = "accuracy_fail"
        ok = (
            failure_stage is None
            and wait is not None
            and wait.ok
            and accurate
        )
        return CaseResult(
            case["id"],
            "vlaServoJ",
            ok,
            "ok" if ok else failure_result,
            metrics={
                **metrics,
                "active_elapsed_sec": float(active_elapsed),
                "actual_send_count": float(send_count),
                "warmup_actual_send_count": float(warmup_send_count),
                "transition_actual_send_count": float(
                    transition_send_count
                ),
                "source_send_count": float(source_send_count),
                "source_actual_send_count": float(source_send_count),
                "return_actual_send_count": float(return_send_count),
                "source_elapsed_sec": float(source_elapsed),
                "actual_send_rate_hz": _actual_send_rate(
                    source_send_count, source_elapsed
                ),
            },
            start_state=start_state,
            final_state=final_state_dict,
            messages=[
                *[
                    json.dumps(item, ensure_ascii=False)
                    for item in notifications
                ],
                *exception_messages,
            ],
            failure_stage=failure_stage,
        )

    def _default_servo_rate(self) -> float:
        return float(self.config.get("sampling", {}).get("servo_rate_hz", 300.0))

    def _sleep_until_sample(self, start_t: float, sample_t: float) -> None:
        remaining = start_t + sample_t - self._monotonic()
        if remaining > 0:
            self._sleep(remaining)

    def _servoj_duration(self, start_q: List[float], target_q: List[float], case: Dict[str, Any]) -> float:
        requested = float(case.get("duration", 0.0))
        max_velocity = float(
            case.get(
                "max_velocity_rad_s",
                self.config.get("safety", {}).get("servoj_max_velocity_rad_s", 0.5),
            )
        )
        if max_velocity <= 0:
            return requested
        max_delta = max(
            abs(target_value - current_value)
            for current_value, target_value in zip(start_q, target_q)
        )
        return max(requested, max_delta / max_velocity)

    def _wait_joint(self, target_q: List[float], indices: Iterable[int], case: Dict[str, Any]) -> WaitResult:
        return wait_for_joint_target(
            self.client,
            target_q,
            indices=indices,
            tolerance_rad=float(self.thresholds.get("joint_max_abs_rad", 0.05)),
            velocity_rad_s=float(self.thresholds.get("stable_velocity_rad_s", 0.02)),
            timeout_sec=self._motion_timeout(case),
            poll_sec=self._state_poll_sec(),
            stable_samples=self._stable_samples(),
            sleep=self._sleep,
            clock=self._monotonic,
        )

    def _wait_pose(self, target_pose: List[float], case: Dict[str, Any]) -> WaitResult:
        return wait_for_pose_target(
            self.client,
            target_pose,
            position_tolerance_m=float(self.thresholds.get("pose_position_m", 0.03)),
            orientation_tolerance_rad=float(self.thresholds.get("pose_orientation_rad", 0.15)),
            timeout_sec=self._motion_timeout(case),
            poll_sec=self._state_poll_sec(),
            stable_samples=self._stable_samples(),
            sleep=self._sleep,
            clock=self._monotonic,
        )

    def _motion_timeout(self, case: Dict[str, Any]) -> float:
        timeouts = self.config.get("timeouts", {})
        if "settle_sec" not in timeouts:
            return 0.0
        return max(0.0, float(case.get("duration", 0.0)) + float(timeouts.get("settle_sec", 0.0)))

    def _state_poll_sec(self) -> float:
        rate = float(self.config.get("sampling", {}).get("state_rate_hz", 20.0))
        return 1.0 / rate if rate > 0 else 0.05

    def _stable_samples(self) -> int:
        rate = float(self.config.get("sampling", {}).get("state_rate_hz", 20.0))
        window = float(self.thresholds.get("stable_window_sec", 0.0))
        return max(1, int(round(rate * window)))

    def _response_timeout(self) -> float:
        return float(self.config.get("timeouts", {}).get("response_sec", 5.0))


def load_yaml_file(path: Path) -> Dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("Missing dependency PyYAML. Install requirements.txt first.") from exc
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def write_csv_report(path: Path, results: Iterable[CaseResult]) -> None:
    rows = [result.as_row() for result in results]
    if not rows:
        return
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json_report(path: Path, results: Iterable[CaseResult]) -> None:
    rows = [result.as_dict() for result in results]
    with path.open("w", encoding="utf-8") as handle:
        json.dump(rows, handle, ensure_ascii=False, indent=2)


def _response_ok(response: Dict[str, Any]) -> bool:
    return response.get("data", {}).get("result") == "success"


def _response_status(response: Dict[str, Any]) -> str:
    return str(response.get("data", {}).get("result", ""))


def _response_fail_result(
    case: Dict[str, Any],
    case_type: str,
    response: Dict[str, Any],
    start_state: Dict[str, Any],
) -> CaseResult:
    status = _response_status(response) or "empty_result"
    return CaseResult(
        case["id"],
        case_type,
        False,
        "response_fail",
        start_state=start_state,
        messages=[status],
        response_status=status,
    )


def _invalid_result(
    case: Dict[str, Any],
    messages: List[str],
    start_state: Dict[str, Any],
) -> CaseResult:
    return CaseResult(
        case["id"],
        str(case["type"]),
        False,
        "validation_fail",
        start_state=start_state,
        messages=messages,
    )


def _runtime_exception_result(
    case: Dict[str, Any],
    exc: Exception,
) -> CaseResult:
    return CaseResult(
        str(case.get("id", "<unknown>")),
        str(case.get("type", "<unknown>")),
        False,
        "runtime_exception",
        messages=[f"{type(exc).__name__}: {exc}"],
        failure_stage="runtime_exception",
        case_metadata=_case_report_metadata(case),
    )


def _invalid_long_result(
    case: Dict[str, Any],
    messages: List[str],
    start_state: Dict[str, Any],
    *,
    failure_stage: str,
) -> CaseResult:
    result = _invalid_result(case, messages, start_state)
    result.metrics = _loop_metrics(float(case["duration"]), LoopStats())
    result.failure_stage = failure_stage
    return result


def _invalid_long_servo_result(
    case: Dict[str, Any],
    case_type: str,
    start_state: Dict[str, Any],
    *,
    segment_duration: float,
    message: str,
    failure_stage: str,
) -> CaseResult:
    return CaseResult(
        case["id"],
        case_type,
        False,
        "validation_fail",
        metrics=_long_servo_metrics(
            segment_duration=segment_duration,
            planned_duration=0.0,
            active_elapsed=0.0,
            active_planned_send_count=0,
            active_actual_send_count=0,
            return_planned_send_count=0,
            return_actual_send_count=0,
            actual_send_rate=0.0,
            completed_segments=0,
            completed_cycles=0,
        ),
        start_state=start_state,
        messages=[message],
        failure_stage=failure_stage,
    )


def _long_servo_metrics(
    *,
    segment_duration: float,
    planned_duration: float,
    active_elapsed: float,
    active_planned_send_count: int,
    active_actual_send_count: int,
    return_planned_send_count: int,
    return_actual_send_count: int,
    actual_send_rate: float,
    completed_segments: int,
    completed_cycles: int,
) -> Dict[str, float]:
    planned_send_count = (
        active_planned_send_count + return_planned_send_count
    )
    actual_send_count = active_actual_send_count + return_actual_send_count
    return {
        "segment_duration_sec": float(segment_duration),
        "planned_duration_sec": float(planned_duration),
        "active_elapsed_sec": float(active_elapsed),
        "active_planned_send_count": float(active_planned_send_count),
        "active_actual_send_count": float(active_actual_send_count),
        "return_planned_send_count": float(return_planned_send_count),
        "return_actual_send_count": float(return_actual_send_count),
        "planned_send_count": float(planned_send_count),
        "actual_send_count": float(actual_send_count),
        "actual_send_rate_hz": float(actual_send_rate),
        "completed_segments": float(completed_segments),
        "completed_cycles": float(completed_cycles),
    }


def _vla_metrics(*, repeat_index: float) -> Dict[str, float]:
    return {
        "repeat_index": float(repeat_index),
        "clamped_value_count": 0.0,
        "clamped_frame_count": 0.0,
        "max_clamp_correction_rad": 0.0,
        "frame_count": 0.0,
        "source_duration_sec": 0.0,
        "warmup_duration_sec": 0.0,
        "transition_duration_sec": 0.0,
        "active_elapsed_sec": 0.0,
        "warmup_planned_send_count": 0.0,
        "warmup_actual_send_count": 0.0,
        "transition_planned_send_count": 0.0,
        "transition_actual_send_count": 0.0,
        "source_planned_send_count": 0.0,
        "source_actual_send_count": 0.0,
        "return_planned_send_count": 0.0,
        "return_actual_send_count": 0.0,
        "planned_send_count": 0.0,
        "actual_send_count": 0.0,
        "actual_send_rate_hz": 0.0,
        "source_send_count": 0.0,
        "source_elapsed_sec": 0.0,
        "return_stability_samples": 0.0,
        "return_stability_elapsed_sec": 0.0,
        "joint_max_abs_rad": 0.0,
        "joint_rms_rad": 0.0,
    }


def _constant_vector_samples(
    values: List[float],
    *,
    duration_sec: float,
    rate_hz: float,
) -> List[VectorSample]:
    duration = float(duration_sec)
    rate = float(rate_hz)
    if not math.isfinite(duration) or duration < 0.0:
        raise ValueError("warmup_duration_sec must be a finite non-negative number")
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError("rate_hz must be a finite positive number")
    if duration == 0.0:
        return []
    step_count = max(1, int(round(duration * rate)))
    return [
        VectorSample(index / rate, list(values))
        for index in range(step_count)
    ]


def _actual_send_rate(send_count: int, elapsed_sec: float) -> float:
    if send_count <= 1 or elapsed_sec <= 0.0:
        return 0.0
    return float((send_count - 1) / elapsed_sec)


def _float_list(values: Iterable[Any]) -> List[float]:
    return [float(value) for value in values]


def _loop_metrics(duration: float, stats: LoopStats) -> Dict[str, float]:
    return {
        "requested_arrival_sec": float(duration),
        "active_elapsed_sec": stats.active_elapsed_sec,
        "completed_segments": float(stats.completed_segments),
        "completed_cycles": float(stats.completed_cycles),
    }


def _segment_stage(segment: Any, index: int) -> str:
    if isinstance(segment, dict):
        for key in ("stage", "id", "name"):
            if key in segment:
                return str(segment[key])
    stage = getattr(segment, "stage", None)
    return str(stage if stage is not None else segment or f"segment_{index}")


def _case_report_metadata(case: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "protocol_type": str(case["type"]),
        "pattern": case.get("pattern"),
        "time_index": case.get("time_index"),
        "repeat_index": case.get("repeat_index"),
        "requested_arrival_sec": case.get("duration"),
        "requested_segment_sec": case.get("segment_duration"),
        "requested_active_duration_sec": case.get("active_duration_sec"),
        "requested_send_rate_hz": case.get("send_rate_hz"),
        "hard_limit_tolerance_rad": case.get(
            "hard_limit_tolerance_rad"
        ),
        "simulation_only": case.get("simulation_only", False),
    }
