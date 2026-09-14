from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from .case_factory import MOVE_TYPES, SERVO_TYPES, VLA_TYPE, expand_cases
from .client import DualArmProtocolClient
from .preflight import run_preflight
from .runner import (
    CaseResult,
    DualArmTestRunner,
    load_yaml_file,
    write_csv_report,
    write_json_report,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "default.yaml"
DEFAULT_CASES = PROJECT_ROOT / "config" / "test_cases.yaml"


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    config = load_yaml_file(args.config)
    cases_doc = load_yaml_file(args.cases)
    overrides: Dict[str, Any] = {"robot": {}}
    if args.robot_ip:
        overrides["robot"]["ip"] = args.robot_ip
    if args.port:
        overrides["robot"]["port"] = args.port
    if args.accid:
        overrides["robot"]["accid"] = args.accid
        overrides["robot"]["auto_accid"] = False
    config = merge_config(config, overrides)
    cases = expand_cases(cases_doc)

    return run_live(
        config,
        cases,
        args.output_dir,
        yes=args.yes,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TRON2 dual-arm JSON protocol tester")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--cases", type=Path, default=DEFAULT_CASES)
    parser.add_argument("--robot-ip", help="Loopback robot IP address, default from config")
    parser.add_argument("--port", type=int, help="WebSocket port, default from config")
    parser.add_argument(
        "--accid",
        help="Robot software serial number, default DACH_TRON2A_001",
    )
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "results")
    parser.add_argument(
        "--run-live",
        action="store_true",
        help="Accepted for compatibility; tests always connect and send commands",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Accepted for compatibility; remote execution is not supported",
    )
    parser.add_argument(
        "--long-run-sim",
        action="store_true",
        help="Accepted for compatibility; simulation-only cases are authorized by default",
    )
    return parser


def merge_config(base: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            child = dict(merged[key])
            child.update({k: v for k, v in value.items() if v is not None})
            merged[key] = child
        elif value is not None:
            merged[key] = value
    return merged


def run_live(
    config: Dict[str, Any],
    cases: List[Dict[str, Any]],
    output_dir: Path,
    *,
    yes: bool = False,
) -> int:
    robot = config.get("robot", {})
    timeouts = config.get("timeouts", {})
    robot_ip = str(robot["ip"])
    semantically_long_cases = [
        case for case in cases if _is_semantically_long_run_case(case)
    ]
    invalid_long_cases = [
        case
        for case in semantically_long_cases
        if case.get("simulation_only") is not True
    ]
    if invalid_long_cases:
        case_ids = ", ".join(
            str(case.get("id", "<unknown>"))
            for case in invalid_long_cases
        )
        print(
            "Long-run simulation configuration error: "
            "simulation_only must be boolean true for "
            f"{case_ids}.",
            flush=True,
        )
        return 2

    has_long_run_sim = bool(semantically_long_cases) or any(
        case.get("simulation_only") is True for case in cases
    )
    if has_long_run_sim and not _is_local_simulation_host(robot_ip):
        print(
            "Long-run simulation cases require robot IP 127.0.0.1.",
            flush=True,
        )
        return 2
    if not _is_local_simulation_host(robot_ip):
        print(
            "Live tests only support local simulation robot IP 127.0.0.1.",
            flush=True,
        )
        return 2

    client = DualArmProtocolClient(
        robot["ip"],
        port=int(robot.get("port", 5000)),
        accid=robot.get("accid"),
        response_timeout_sec=float(timeouts.get("response_sec", 5.0)),
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    completed_results: List[CaseResult] = []
    try:
        print(f"Connecting to {client.url}", flush=True)
        client.connect(timeout_sec=float(timeouts.get("connect_sec", 5.0)))
        if robot.get("auto_accid", True) and not client.accid:
            client.wait_for_accid(timeout_sec=float(timeouts.get("connect_sec", 5.0)))
        preflight = run_preflight(client, config)
        if not preflight.ok:
            _write_preflight_report(output_dir / "preflight_summary.json", preflight.as_row())
            print(f"Preflight failed: {preflight.code} - {preflight.message}", flush=True)
            print(f"Reports written to {output_dir}", flush=True)
            return 2
        _write_preflight_report(output_dir / "preflight_summary.json", preflight.as_row())
        print("Preflight passed.", flush=True)
        case_label = "case" if len(cases) == 1 else "cases"
        print(f"Running {len(cases)} live {case_label}", flush=True)
        runner = DualArmTestRunner(client, config)

        def save_result(result: CaseResult) -> None:
            completed_results.append(result)
            _write_case_reports(output_dir, completed_results)

        results = runner.run_all(cases, on_result=save_result)
        _write_case_reports(output_dir, results)
        passed = sum(1 for result in results if result.ok)
        print(f"Live run complete: {passed}/{len(results)} passed", flush=True)
        print(f"Reports written to {output_dir}", flush=True)
        return 0 if all(result.ok for result in results) else 2
    except KeyboardInterrupt:
        if completed_results:
            _write_case_reports(output_dir, completed_results)
        print(
            f"Live run interrupted; preserved {len(completed_results)} completed results.",
            flush=True,
        )
        if completed_results:
            print(f"Reports written to {output_dir}", flush=True)
        return 130
    finally:
        client.close()


def _is_semantically_long_run_case(case: Dict[str, Any]) -> bool:
    case_type = case.get("type")
    return case_type == VLA_TYPE or (
        case_type in MOVE_TYPES | SERVO_TYPES
        and "active_duration_sec" in case
    )


def _is_local_simulation_host(robot_ip: str) -> bool:
    return robot_ip.strip() == "127.0.0.1"


def _write_preflight_report(path: Path, report: Dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)


def _write_case_reports(
    output_dir: Path,
    results: Iterable[CaseResult],
) -> None:
    snapshot = list(results)
    csv_path = output_dir / "cases_summary.csv"
    json_path = output_dir / "cases_summary.json"
    csv_temp = csv_path.with_name(csv_path.name + ".tmp")
    json_temp = json_path.with_name(json_path.name + ".tmp")
    try:
        if snapshot:
            write_csv_report(csv_temp, snapshot)
        else:
            csv_temp.write_text("", encoding="utf-8")
        write_json_report(json_temp, snapshot)
        csv_temp.replace(csv_path)
        json_temp.replace(json_path)
    finally:
        for temp_path in (csv_temp, json_temp):
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
