from __future__ import annotations

import math
from typing import Any, Dict, List, Set


MOVE_TYPES = {"moveJ", "moveH", "moveP"}
SERVO_TYPES = {"servoJ", "servoP"}
VLA_TYPE = "vlaServoJ"


def arrival_times(count: int = 20, minimum: float = 1.0, maximum: float = 5.0) -> List[float]:
    if (
        isinstance(count, bool)
        or not isinstance(count, int)
        or count < 2
        or not math.isfinite(minimum)
        or not math.isfinite(maximum)
        or minimum <= 0
        or maximum <= minimum
    ):
        raise ValueError("invalid arrival time range")

    values = [
        minimum + (maximum - minimum) * index / (count - 1)
        for index in range(count)
    ]
    values[0] = minimum
    values[-1] = maximum
    if any(left >= right for left, right in zip(values, values[1:])):
        raise ValueError("arrival time range is too narrow for unique values")
    return values


def expand_cases(document: Dict[str, Any]) -> List[Dict[str, Any]]:
    explicit = [dict(case) for case in document.get("cases", [])]
    generated: List[Dict[str, Any]] = []
    for spec in document.get("generators", []):
        generated.extend(expand_generator(spec))
    return explicit + generated


def expand_generator(spec: Dict[str, Any]) -> List[Dict[str, Any]]:
    case_type = spec.get("type")
    if not case_type:
        raise ValueError("generator requires type")
    if case_type == VLA_TYPE:
        return _expand_vla_generator(spec)
    if case_type not in MOVE_TYPES | SERVO_TYPES:
        raise ValueError(f"unsupported generator type: {case_type}")

    entries = _generator_entries(spec)
    if "groups" in spec:
        count = spec.get("count", len(entries))
        if count > len(entries):
            raise ValueError("generator count cannot exceed groups")
        entries = entries[:count]
    else:
        count = spec.get("count", 20)
    time_range = spec.get("time_range_sec", [1.0, 5.0])
    if not isinstance(time_range, (list, tuple)) or len(time_range) != 2:
        raise ValueError("time_range_sec must contain minimum and maximum")
    times = arrival_times(count, time_range[0], time_range[1])
    common = _common_fields(
        spec,
        {
            "count",
            "time_range_sec",
            "patterns",
            "pattern_count",
            "id_prefix",
            "groups",
        },
    )
    id_prefix = str(spec.get("id_prefix", f"{case_type}_long"))
    time_field = "duration" if case_type in MOVE_TYPES else "segment_duration"

    return [
        {
            **common,
            **_entry_fields(entries[index % len(entries)]),
            "id": f"{id_prefix}_{index + 1:02d}",
            "type": case_type,
            "time_index": index,
            time_field: arrival_time,
        }
        for index, arrival_time in enumerate(times)
    ]


def _expand_vla_generator(spec: Dict[str, Any]) -> List[Dict[str, Any]]:
    count = spec.get("count", 20)
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError("VLA generator count must be a positive integer")
    if not spec.get("states_file"):
        raise ValueError("VLA generator requires states_file")

    common = _common_fields(
        spec,
        {"count", "id_prefix", "preserve_timestamps"},
    )
    id_prefix = str(spec.get("id_prefix", "vlaServoJ_long"))
    return [
        {
            **common,
            "id": f"{id_prefix}_{index + 1:02d}",
            "type": VLA_TYPE,
            "repeat_index": index,
            # VLA repeats always retain the source file's original timeline.
            "preserve_timestamps": True,
        }
        for index in range(count)
    ]


def _generator_entries(spec: Dict[str, Any]) -> List[Any]:
    if "groups" in spec:
        groups = list(spec["groups"])
        if not groups:
            raise ValueError("groups must not be empty")
        for group in groups:
            if not isinstance(group, dict) or "waypoints" not in group:
                raise ValueError("group mapping requires waypoints")
            if not group["waypoints"]:
                raise ValueError("group waypoints must not be empty")
        return groups
    return _patterns(spec)


def _patterns(spec: Dict[str, Any]) -> List[Any]:
    if "patterns" in spec:
        patterns = list(spec["patterns"])
        if not patterns:
            raise ValueError("patterns must not be empty")
        return patterns

    if "pattern_count" not in spec:
        raise ValueError("generator requires groups, patterns, or pattern_count")
    pattern_count = spec["pattern_count"]
    if (
        isinstance(pattern_count, bool)
        or not isinstance(pattern_count, int)
        or pattern_count < 1
    ):
        raise ValueError("pattern_count must be a positive integer")
    return [f"pattern_{index + 1:02d}" for index in range(pattern_count)]


def _common_fields(spec: Dict[str, Any], excluded: Set[str]) -> Dict[str, Any]:
    return {key: value for key, value in spec.items() if key not in excluded}


def _entry_fields(entry: Any) -> Dict[str, Any]:
    if isinstance(entry, dict) and "waypoints" in entry:
        return dict(entry)
    return _pattern_fields(entry)


def _pattern_fields(pattern: Any) -> Dict[str, Any]:
    if not isinstance(pattern, dict):
        return {"pattern": pattern}
    name = pattern.get("name")
    if not isinstance(name, str) or not name:
        raise ValueError("pattern mapping requires a non-empty name")
    return {
        "pattern": name,
        **{key: value for key, value in pattern.items() if key != "name"},
    }
