from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO_PATH = PACKAGE_DIR / "DOCS" / "default_scenario.yaml"

DEFAULT_COLOR_PALETTE = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
]

DRIVE_MODE_ALIASES = {
    "ATO": "ATO",
    "AUTOMATIC": "ATO",
    "LMD": "LMD",
    "MCS": "LMD",
    "MTC": "LMD",
    "MANUAL": "LMD",
    "CMD25": "CMD25",
    "RM": "CMD25",
    "RM25": "CMD25",
    "RESTRICTED": "CMD25",
    "RESTRICTED_MANUAL": "CMD25",
}

DEFAULT_SCENARIO: Dict[str, Any] = {
    "name": "Cau hinh tuyen mac dinh",
    "display": {
        "window_title": "CBTC ATC Simulation - Single Monitor",
        "track_min_m": -220.0,
        "track_max_m": 2000.0,
        "track_labels": [0, 500, 1000, 1500, 2000],
    },
    "train_defaults": {
        "length_m": 60.0,
        "mass_kg": 291600.0,
        "car_count": 4,
    },
    "track": {
        "segments": [
            {"start_m": 0.0, "end_m": 500.0, "gradient": 0.02, "psr_kmh": 30.0},
            {"start_m": 500.0, "end_m": 1000.0, "gradient": 0.05, "psr_kmh": 40.0},
            {"start_m": 1000.0, "end_m": 1500.0, "gradient": -0.06, "psr_kmh": 35.0},
            {"start_m": 1500.0, "end_m": 2000.0, "gradient": -0.01, "psr_kmh": 25.0},
        ]
    },
    "scheduled_stops": [],
    "trains": [],
    "source_trains": [],
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _default_track_labels(track_min_m: float, track_max_m: float) -> List[int]:
    if track_max_m <= track_min_m:
        return [int(track_min_m)]
    span = track_max_m - track_min_m
    step = max(100.0, round(span / 4.0 / 100.0) * 100.0)
    labels: List[int] = []
    current = 0.0
    while current <= track_max_m:
        if current >= track_min_m:
            labels.append(int(current))
        current += step
    if not labels:
        labels.append(int(track_min_m))
    return labels


def _normalize_track_segments(raw_segments: List[Dict[str, Any]]) -> List[tuple[float, float, float, float]]:
    if not raw_segments:
        raise ValueError("Scenario must define at least one track segment.")

    segments: List[tuple[float, float, float, float]] = []
    for idx, segment in enumerate(raw_segments):
        try:
            start_m = float(segment["start_m"])
            end_m = float(segment["end_m"])
            gradient = float(segment.get("gradient", 0.0))
            psr_kmh = float(segment["psr_kmh"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid track segment at index {idx}.") from exc
        if end_m <= start_m:
            raise ValueError(f"Track segment {idx} must have end_m > start_m.")
        segments.append((start_m, end_m, gradient, psr_kmh))
    segments.sort(key=lambda item: item[0])
    return segments


def _normalize_balises(raw_balises: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    balises: List[Dict[str, Any]] = []
    seen_ids = set()
    for idx, balise in enumerate(raw_balises):
        try:
            pos_m = float(balise["pos_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid balise definition at index {idx}.") from exc
        balise_id = str(balise.get("id", f"BL_{idx + 1:03d}"))
        if balise_id in seen_ids:
            raise ValueError(f"Duplicate balise id '{balise_id}' in scenario.")
        seen_ids.add(balise_id)
        balises.append({"id": balise_id, "pos_m": pos_m})
    balises.sort(key=lambda item: item["pos_m"])
    return balises


def _normalize_trains(raw_trains: List[Dict[str, Any]], defaults: Dict[str, Any]) -> List[Dict[str, Any]]:
    trains: List[Dict[str, Any]] = []
    seen_ids = set()
    for idx, train in enumerate(raw_trains):
        try:
            train_id = str(train["id"])
            start_pos = float(train["start_pos"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid train definition at index {idx}.") from exc
        if train_id in seen_ids:
            raise ValueError(f"Duplicate train id '{train_id}' in scenario.")
        seen_ids.add(train_id)
        raw_drive_mode = str(train.get("drive_mode", defaults.get("drive_mode", "ATO"))).upper()
        drive_mode = DRIVE_MODE_ALIASES.get(raw_drive_mode)
        if drive_mode is None:
            raise ValueError(f"Invalid drive_mode '{raw_drive_mode}' for train '{train_id}'.")
        dcs_mute_windows = []
        for win_idx, window in enumerate(train.get("dcs_mute_windows", [])):
            try:
                start_s = float(window["start_s"])
                end_s = float(window["end_s"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"Invalid DCS mute window {win_idx} for train '{train_id}'.") from exc
            if end_s <= start_s:
                raise ValueError(f"DCS mute window {win_idx} for train '{train_id}' must have end_s > start_s.")
            dcs_mute_windows.append({"start_s": start_s, "end_s": end_s})
        trains.append(
            {
                "id": train_id,
                "start_pos": start_pos,
                "length_m": float(train.get("length_m", defaults["length_m"])),
                "mass_kg": float(train.get("mass_kg", defaults["mass_kg"])),
                "car_count": int(train.get("car_count", defaults.get("car_count", 4))),
                "drive_mode": drive_mode,
                "requested_drive_mode": raw_drive_mode,
                "max_ato_speed_kmh": float(train.get("max_ato_speed_kmh", defaults.get("max_ato_speed_kmh", 70.0))),
                "max_manual_speed_kmh": float(train.get("max_manual_speed_kmh", defaults.get("max_manual_speed_kmh", 45.0))),
                "dcs_mute_windows": dcs_mute_windows,
                "color": train.get("color"),
            }
        )
    return trains


def _normalize_scheduled_stops(raw_stops: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    stops: List[Dict[str, Any]] = []
    for idx, stop in enumerate(raw_stops):
        try:
            pos_m = float(stop["pos_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid scheduled stop at index {idx}.") from exc
        normalized = {
            "name": str(stop.get("name", f"STOP_{idx + 1}")),
            "pos_m": pos_m,
            "length_m": float(stop.get("length_m", 160.0)),
            "capacity": int(stop.get("capacity", 3)),
        }
        stops.append(normalized)
    stops.sort(key=lambda item: item["pos_m"])
    return stops


def _normalize_tsr_zones(raw_zones: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Normalize and validate Temporary Speed Restriction zones."""
    zones: List[Dict[str, Any]] = []
    for idx, zone in enumerate(raw_zones):
        try:
            start_m = float(zone["start"])
            end_m = float(zone["end"])
            speed_kmh = float(zone["speed"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid TSR zone at index {idx}.") from exc
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m <= start_m:
            raise ValueError(f"TSR zone {idx} must have end > start.")
        if speed_kmh <= 0.0:
            raise ValueError(f"TSR zone {idx} must have speed > 0.")
        zones.append({
            "start": start_m,
            "end": end_m,
            "speed": speed_kmh,
        })
    zones.sort(key=lambda item: item["start"])
    return zones


def normalize_scenario(data: Dict[str, Any], source_path: str | None = None) -> Dict[str, Any]:
    merged = _deep_merge(DEFAULT_SCENARIO, data)
    display = merged.get("display", {})
    track_profile = _normalize_track_segments(merged["track"]["segments"])
    track_end_m = track_profile[-1][1]
    source_starts = [float(source.get("start_m", 0.0)) for source in merged.get("source_trains", [])]
    train_starts = [float(train["start_pos"]) for train in merged.get("trains", [])]
    min_known_pos = min([0.0] + train_starts + source_starts)
    track_min_m = float(display.get("track_min_m", min_known_pos))
    track_max_m = float(display.get("track_max_m", track_end_m))
    track_labels = display.get("track_labels") or _default_track_labels(track_min_m, track_max_m)
    train_defaults = {
        "length_m": float(merged["train_defaults"]["length_m"]),
        "mass_kg": float(merged["train_defaults"]["mass_kg"]),
        "car_count": int(merged["train_defaults"].get("car_count", 4)),
        "drive_mode": str(merged["train_defaults"].get("drive_mode", "ATO")),
        "max_ato_speed_kmh": float(merged["train_defaults"].get("max_ato_speed_kmh", 70.0)),
        "max_manual_speed_kmh": float(merged["train_defaults"].get("max_manual_speed_kmh", 45.0)),
    }
    trains = _normalize_trains(merged["trains"], train_defaults)
    scheduled_stops = _normalize_scheduled_stops(merged.get("scheduled_stops", []))
    tsr_zones = _normalize_tsr_zones(merged.get("tsr_zones", []))
    raw_headway = merged.get("headway", {}) if isinstance(merged.get("headway", {}), dict) else {}
    headway = {
        "mode": "fixed",
        "target_headway_s": float(raw_headway.get("target_headway_s", raw_headway.get("fixed_headway_s", 180.0))),
    }
    raw_communication = merged.get("communication", {}) if isinstance(merged.get("communication", {}), dict) else {}
    communication = {
        "use_vital_position_report_for_zc": bool(raw_communication.get("use_vital_position_report_for_zc", True)),
    }
    raw_localization = merged.get("localization", {}) if isinstance(merged.get("localization", {}), dict) else {}
    balises = _normalize_balises(raw_localization.get("balises", []))
    return {
        "name": str(merged.get("name", "Line Configuration")),
        "source_path": source_path,
        "window_title": str(display.get("window_title", DEFAULT_SCENARIO["display"]["window_title"])),
        "track_min_m": track_min_m,
        "track_max_m": track_max_m,
        "track_labels": [int(label) for label in track_labels],
        "block_mode": "moving_block",
        "track_profile": track_profile,
        "track_end_m": track_end_m,
        "train_defaults": train_defaults,
        "scheduled_stops": scheduled_stops,
        "trains": trains,
        "source_trains": deepcopy(merged.get("source_trains", [])),
        "headway": headway,
        "communication": communication,
        "balises": balises,
        "headway_config_present": True,
        "line_conditions": deepcopy(merged.get("line_conditions", [])),
        "tsr_zones": tsr_zones,
        "radio_access_points": deepcopy(merged.get("radio_access_points", [])),
        "radio_physical": deepcopy(merged.get("radio_physical", {})),
        "color_palette": deepcopy(DEFAULT_COLOR_PALETTE),
    }


def load_scenario(path: str | Path | None = None) -> Dict[str, Any]:
    if path is None:
        path = DEFAULT_SCENARIO_PATH
    scenario_path = Path(path)
    with scenario_path.open("r", encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh) or {}
    if not isinstance(loaded, dict):
        raise ValueError("Scenario YAML root must be a mapping.")
    return normalize_scenario(loaded, str(scenario_path))


