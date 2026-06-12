from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict


@dataclass(frozen=True)
class MovementAuthorityMessage:
    train_id: str
    eoa_m: float
    psr_kmh: float
    gradient: float = 0.0
    next_speed_limit_kmh: float = 0.0
    next_speed_limit_dist_m: float = float("inf")
    issued_time_s: float = 0.0
    reason: str = ""

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PositionReportMessage:
    train_id: str
    safe_front_m: float
    safe_rear_m: float
    speed_mps: float
    direction: str
    localization_uncertainty_m: float
    train_integrity_ok: bool
    timestamp_ms: int
    trip_mode: bool = False
    trip_protect_rear_m: float = 0.0
    protection_zone_id: str | None = None
    protection_lane: int = 0
    active_scheduled_stop: Dict[str, Any] | None = None

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TrainStatusMessage:
    train_id: str
    position_m: float
    speed_mps: float
    mode: str
    atp_state: str
    ato_state: str
    door_state: str
    brake_state: str
    fault_flags: Dict[str, bool] = field(default_factory=dict)
    timestamp_ms: int = 0
    freshness: str = "FRESH"
    length_m: float = 0.0
    safe_front_m: float = 0.0
    safe_rear_m: float = 0.0
    color: str = ""
    protection_zone_id: str | None = None
    protection_lane: int = 0
    active_scheduled_stop: Dict[str, Any] | None = None
    station_lane: int | None = None
    departure_hold: bool = False
    eoa_m: float = 0.0
    eoa_reason: str = ""
    distance_to_eoa_m: float = 0.0
    constraint_type: str = "NONE"
    constraint_target_speed_kmh: float = 0.0
    distance_to_constraint_m: float = float("inf")
    direction: str = "FORWARD"
    odometry_uncertainty_m: float = 0.0
    speed_curves_kmh: Dict[str, float] = field(default_factory=dict)
    atp_action: str = ""
    atp_alert: str = ""
    ato_target_speed_kmh: float = 0.0
    ato_recovery_state: str = "NORMAL"
    mode_transition_reason: str = ""
    psr_kmh: float = 0.0
    limit_ahead_speed_kmh: float = 0.0
    limit_ahead_dist_m: float = float("inf")
    active_controller: str = "CC_A"
    standby_controller: str = "CC_B"
    cc_a_status: str = "HEALTHY"
    cc_b_status: str = "HEALTHY"
    cc_a_fault_reason: str = ""
    cc_b_fault_reason: str = ""
    fault_reason: str = ""
    controller_switch_count: int = 0
    switch_counter: int = 0
    last_switch_time: float = 0.0
    rolling_stock_status: Dict[str, Any] = field(default_factory=dict)

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class WaysideStatusMessage:
    track_profile: list
    track_min_m: float
    track_max_m: float
    track_end_m: float
    track_labels: list
    scheduled_stops: list
    line_conditions: list
    source_trains: list
    balises: list
    radio_access_points: list
    timestamp_ms: int = 0

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ZcStatusMessage:
    zc_status: Dict[str, Any]
    tsr_zones: list
    secondary_detection_sections: list = field(default_factory=list)
    protection_zones: list = field(default_factory=list)
    virtual_obstacles: list = field(default_factory=list)
    timestamp_ms: int = 0

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class StationStatusMessage:
    station_route_states: list
    point_states: list = field(default_factory=list)
    timestamp_ms: int = 0

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DcsStatusMessage:
    dcs_transport_state: Dict[str, Any]
    timestamp_ms: int = 0

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class AtsOperationCommandMessage:
    command: str
    train_id: str
    value: Any = None
    reason: str = ""

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DcsHealthMessage:
    red_state: str
    blue_state: str
    active_path: str
    latency_ms_avg: float
    packet_loss_count: int
    timeout_count: int
    freshness: str = "FRESH"

    def to_payload(self) -> Dict[str, Any]:
        return asdict(self)
