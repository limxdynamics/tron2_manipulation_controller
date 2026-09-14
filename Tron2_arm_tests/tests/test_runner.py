from __future__ import annotations

import csv
import json

from dual_arm_protocol_tests.runner import (
    CaseResult,
    DualArmTestRunner,
    write_csv_report,
    write_json_report,
)


def test_run_segment_loop_tracks_cycles_with_controlled_clock() -> None:
    now = 0.0

    def monotonic() -> float:
        return now

    def execute_segment(_: str) -> bool:
        nonlocal now
        now += 1.0
        return True

    runner = DualArmTestRunner(
        client=None,
        monotonic=monotonic,
        sleep=lambda _: None,
    )

    stats = runner.run_segment_loop(
        active_duration_sec=2.0,
        segments=["first", "second"],
        execute_segment=execute_segment,
    )

    assert stats.active_elapsed_sec == 2.0
    assert stats.completed_segments == 2
    assert stats.completed_cycles == 1
    assert stats.failure_stage is None


def test_run_segment_loop_records_failure_stage() -> None:
    now = 0.0

    def monotonic() -> float:
        return now

    def execute_segment(_: dict) -> bool:
        nonlocal now
        now += 0.5
        return False

    runner = DualArmTestRunner(
        client=None,
        monotonic=monotonic,
        sleep=lambda _: None,
    )

    stats = runner.run_segment_loop(
        active_duration_sec=1.0,
        segments=[{"stage": "waypoint_1"}],
        execute_segment=execute_segment,
    )

    assert stats.completed_segments == 0
    assert stats.failure_stage == "waypoint_1"


def test_run_all_stops_after_first_failure_by_default() -> None:
    class FailingRunner(DualArmTestRunner):
        def run_case(self, case: dict) -> CaseResult:
            return CaseResult(case["id"], case["type"], False, "failed")

    runner = FailingRunner(client=None)

    results = runner.run_all(
        [
            {"id": "first", "type": "moveJ"},
            {"id": "second", "type": "moveJ"},
        ]
    )

    assert [result.case_id for result in results] == ["first"]


def test_case_result_and_reports_are_serialized(tmp_path) -> None:
    result = CaseResult(
        "case_1",
        "moveJ",
        True,
        "ok",
        metrics={"joint_max_abs_rad": 0.01},
        start_state={"joint_q": [0.0]},
        final_state={"joint_q": [0.01]},
        response_status="success",
        case_metadata={"time_index": 0},
    )
    csv_path = tmp_path / "cases_summary.csv"
    json_path = tmp_path / "cases_summary.json"

    write_csv_report(csv_path, [result])
    write_json_report(json_path, [result])

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        csv_rows = list(csv.DictReader(handle))
    json_rows = json.loads(json_path.read_text(encoding="utf-8"))

    assert csv_rows[0]["id"] == "case_1"
    assert csv_rows[0]["response_status"] == "success"
    assert json_rows[0]["metrics"] == {"joint_max_abs_rad": 0.01}
