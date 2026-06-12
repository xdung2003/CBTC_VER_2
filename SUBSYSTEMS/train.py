from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from typing import Any, Dict, List

from CONFIG.config import DT, MAX_JERK_MS3
from SUBSYSTEMS.ato import ATOPilotingEngine
from SUBSYSTEMS.atp import ATPEnvelopeEngine
from SUBSYSTEMS.control_common import (
    ATO_PID_INT_LIMIT_MS,
    ATO_TARGET_DROP_RATE_KMH_S,
    AUTO_DOCKING_JOG_WINDOW_M,
    BALISE_ERROR_MAX_M,
    BALISE_POS_UNCERT_M,
    BALISE_SPACING_M,
    BEACON_LOCK_RELEASE_MARGIN_M,
    BRAKE_BUILDUP_S,
    COAST_BASE_DECEL,
    COAST_SPEED_GAIN,
    CREEP_MAX_SPEED_KMH,
    CURVE_DISPLAY_DROP_RATE_KMH_S,
    CURVE_DISPLAY_RISE_RATE_KMH_S,
    CURVE_EPS_KMH,
    DCS_STARTUP_GRACE_S,
    DCS_TIMEOUT_S,
    DOWNHILL_P_BUFFER_KMH_PER_GRAD,
    FINAL_APPROACH_MIN_SPEED_KMH,
    G,
    JOG_MAX_DIST_M,
    JOG_PROFILE_ACCEL_MS2,
    JOG_SPEED_KMH,
    JOG_STATE_ACTIVE,
    JOG_STATE_COMPLETED,
    JOG_STATE_FAILED_LOCKED,
    JOG_STATE_IDLE,
    JOG_STATE_REQUESTED,
    JOG_WINDOW_EPS_M,
    LOW_SPEED_ATP_GUARD_KMH,
    MANUAL_JOG_WINDOW_M,
    ODOMETER_ERROR_RATE,
    POS_UNCERT_M,
    PRECISE_STOP_POS_UNCERT_M,
    PRECISE_STOP_SERVICE_BAND_M,
    RELEASE_HANDOVER_START_M,
    RELEASE_SPEED_KMH,
    RELEASE_ZONE_M,
    ROLLBACK_PROTECT_M,
    SBI_RELEASE_HYST_KMH,
    SBI_VIOLATION_EPS_KMH,
    SPEED_FILTER_TAU_S,
    STANDSTILL_DRIFT_M,
    STANDSTILL_SPEED_EPS,
    STATION_BALISE_ERROR_MAX_M,
    STATION_BALISE_SPACING_M,
    STATION_BALISE_ZONE_M,
    STATION_POS_UNCERT_M,
    STOP_ACCURACY_TOL_M,
    STOP_BEACON_OFFSET_M,
    STOP_EBI_SUPERVISION_FLOOR_KMH,
    STOP_TARGET_OFFSET_M,
    ato_pid_gains,
    ato_tracking_margin_ms,
    low_pass_step,
)
from SUBSYSTEMS.dcs import RedundantOnboardControlCenter
from SUBSYSTEMS.signalling import (
    ATP_MIN_DECEL_MS2,
    SafeMovementPacket,
    STOP_SVL_OFFSET_M,
    stopping_distance_with_buildup,
)
from SUBSYSTEMS.physics import (
    kmh_to_ms,
    limit_jerk,
    ms_to_kmh,
    running_resistance_accel_ms2,
)


@dataclass
class ControlChannel:
    name: str
    ato_ok: bool = True
    atp_ok: bool = True
    controller_ok: bool = True
    active: bool = False
    standby: bool = False
    fault_reason: str = ""
    last_switch_time: float = 0.0

    def refresh(self):
        faults = []
        if not self.atp_ok:
            faults.append("ATP FAULT")
        if not self.ato_ok:
            faults.append("ATO FAULT")
        self.controller_ok = not faults
        self.fault_reason = " / ".join(faults)

    @property
    def status(self) -> str:
        return "HEALTHY" if self.controller_ok else self.fault_reason


def train_color(train_cfg: Dict[str, float | str | None], index: int, palette: List[str]) -> str:
    color = train_cfg.get("color")
    if isinstance(color, str) and color:
        return color
    return palette[index % len(palette)]


class Train:
    def __init__(self, train_cfg: Dict[str, float | str | None]):
        train_id = str(train_cfg["id"])
        start_pos = float(train_cfg["start_pos"])
        self.id = train_id
        self.pos = start_pos
        self.reported_pos = start_pos
        self.speed = 0.0
        self.filtered_speed = 0.0
        self.estimated_speed = 0.0
        self.vital_speed = 0.0
        self.length = float(train_cfg["length_m"])
        self.car_count = max(1, int(train_cfg.get("car_count", 4) or 4))
        self.virtual_integrity_links = [True for _ in range(max(0, self.car_count - 1))]
        self.mass = float(train_cfg["mass_kg"])
        self.drive_mode = str(train_cfg.get("drive_mode", "ATO")).upper()
        self.requested_drive_mode = str(train_cfg.get("requested_drive_mode", self.drive_mode)).upper()
        self.mode_transition_reason = ""
        self.dcs_degraded_requested = False
        self.max_ato_speed_kmh = float(train_cfg.get("max_ato_speed_kmh", 70.0))
        self.max_manual_speed_kmh = float(train_cfg.get("max_manual_speed_kmh", 45.0))
        self.dcs_mute_windows = [dict(window) for window in train_cfg.get("dcs_mute_windows", [])]
        self.dcs_muted = False
        self.track_profile = train_cfg["track_profile"]
        self.balises = self._normalize_balises(train_cfg.get("balises", []))
        self.color = str(train_cfg["color"])
        self.eoa = 0.0
        self.psr_kmh = 25.0
        self.gradient = 0.0
        self.commanded_stop = False
        self.emergency_stop = False
        self.ato_state = "ATO_IDLE"
        self.atp_state = "ATP_OK"
        self.atp_brake = "NONE"
        self.atp_alert = "OK"
        self.traction_cutoff = False
        self.service_brake_latch = False
        self.emg_latch = False
        self.emg_ack = False
        self.emergency_recovery_hold = False
        self.trip_mode = False
        self.trip_reason = ""
        self.trip_protect_pos = start_pos
        self.trip_protect_rear_pos = start_pos - self.length
        self.atp_action = ""
        self.curves = {
            "P": 0.0,
            "W": 0.0,
            "SBD": 0.0,
            "EBD": 0.0,
        }
        self.hidden_curves = {
            "I": 0.0,
            "OFF": 0.0,
            "SBI": 0.0,
            "EBI": 0.0,
        }
        self.raw_curves = dict(self.curves)
        self.raw_hidden_curves = dict(self.hidden_curves)
        self._display_curves_initialized = False
        self._previous_display_curves_for_smoothing: Dict[str, float] | None = None
        self._previous_display_hidden_for_smoothing: Dict[str, float] | None = None
        self.curve_mode = "CEILING"
        self.cutoff_threshold = 0.0
        self.distance_to_eoa = 0.0
        self.safe_front_end_pos = start_pos
        self.ato_target_speed = 0.0
        self.ato_curve_speed = 0.0
        self.ato_piloting_speed = 0.0
        self.margin_dyn_m = 0.0
        self.atp_service_brake_decel = 0.0
        self.atp_emergency_brake_decel = 0.0
        self.limit_ahead_speed_kmh = 0.0
        self.limit_ahead_dist = float("inf")
        self.constraint_type = "NONE"
        self.constraint_target_speed_kmh = 0.0
        self.distance_to_constraint_m = float("inf")
        self.last_balise_pos = self._previous_balise_pos(start_pos)
        self.next_balise_pos = self._next_balise_pos(start_pos)
        self.has_balise_fix = False
        self.pos_error_m = 0.0
        self.odometer_error_sign = 1.0 if (sum(ord(ch) for ch in self.id) % 2 == 0) else -1.0
        self.prev_pos = start_pos
        self.rollback_protection = False
        self.door_open_allowed = False
        self.headway_time_s = None
        self.headway_target_s = 0.0
        self.headway_planned_dispatch_s = None
        self.headway_dispatch_delay_s = 0.0
        self.headway_dispatch_released = False
        self.headway_actual_dispatched = False
        self.headway_hold_reason = ""
        self.protection_zone_id = None
        self.protection_lane = 0
        self.source_name = train_cfg.get("source_name")
        self.source_lane = None
        self.schedule_service_id = train_cfg.get("schedule_service_id")
        self.schedule_profile = str(train_cfg.get("schedule_profile", "") or "")
        self.schedule_records = [dict(record) for record in train_cfg.get("schedule_records", [])]
        self.schedule_planned_dispatch_s = train_cfg.get("schedule_planned_dispatch_s")
        self.station_lane = None
        self.assigned_station_id = None
        self.assigned_station_line_id = None
        self.departure_hold = False
        self.standstill_required = False
        self.standstill_anchor_pos = start_pos
        self.stop_target_pos = start_pos
        self.stop_beacon_pos = start_pos
        self.stop_beacon_seen = False
        self.beacon_lock_stop_pos = None
        self.jog_active = False
        self.jog_used = False
        self.manual_jog_requested = False
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_state = JOG_STATE_IDLE
        self.jog_used_for_current_stop = False
        self.current_stop_id = None
        self.active_stop_key = None
        self.jog_used_stop_key = None
        self.jog_event_counts = {}
        self.jog_stop_target_pos = None
        self.jog_start_pos = start_pos
        self.jog_profile_distance_m = 0.0
        self.prev_commanded_stop = False
        self.release_active = False
        self.release_speed_kmh = 0.0
        self.release_blend = 0.0
        self.release_p_target_ms = 0.0
        self.release_p_release_ms = 0.0
        self.prev_accel = 0.0
        self.zero_speed_detected = True
        self.standstill_monitoring = False
        self.rollback_monitoring = True
        self.door_authorized = False
        self.ato_hold_active = True
        self.ato_door_mode = "LOCKED"
        self.precise_stop_state = "ALIGNED"
        self.ato_brake_prepare = False
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.scheduled_stops = [dict(stop) for stop in train_cfg.get("scheduled_stops", [])]
        self.next_scheduled_stop_idx = 0
        self.active_scheduled_stop = None
        self.dwell_remaining_s = 0.0
        self.station_state = "COMPLETED_STOP"
        self.station_state_stop_key = None
        self.last_station_state_reason = ""
        self.last_station_idx = None
        self.assigned_platform = None
        self.last_dispatched_eoa = None
        self.last_dispatched_eoa_reason = ""
        self.station_reject_reason = ""
        self.beacon_position_locked = False
        self.safe_packet_age_s = 0.0
        self.safe_packet_valid = True
        self.ma_freshness = "FRESH"
        self.vital_packet_result = "ACCEPTED"
        self.vital_packet_reason = ""
        self.dcs_fault_active = False
        self.ato_fault_active = False
        self.ato_recovery_state = "NORMAL"
        self.atp_fault_active = False
        self.integrity_fault_active = False
        self.collision_latched = False
        self.collision_partner_id = ""
        self.collision_overlap_m = 0.0
        self.analytics_distance_m = 0.0
        self.analytics_traction_work_j = 0.0
        self.analytics_brake_work_j = 0.0
        self.runtime_traction_force_n = 0.0
        self.runtime_brake_force_n = 0.0
        self.runtime_used_legacy_fallback = False
        self.ato_brake_mode = "none"
        self.last_sim_time_s = 0.0
        self.event_records = deque(maxlen=240)
        self.pending_event_records = deque(maxlen=80)
        self._last_atp_service_active = False
        self._last_atp_emergency_active = False
        self._last_door_authorized = False
        self._low_speed_ebi_counter = 0
        self._low_speed_guard_active = False
        self.cc_a = ControlChannel("CC_A", active=True)
        self.cc_b = ControlChannel("CC_B", standby=True)
        self.active_controller = self.cc_a
        self.standby_controller = self.cc_b
        self.controller_switch_count = 0
        self.last_controller_switch_time = 0.0
        self.cc = RedundantOnboardControlCenter(self.id, DCS_TIMEOUT_S, DCS_STARTUP_GRACE_S)
        self.atp_engine = ATPEnvelopeEngine()
        self.ato_engine = ATOPilotingEngine()

    @staticmethod
    def _normalize_balises(raw_balises: Any) -> List[float]:
        positions: List[float] = []
        for item in raw_balises or []:
            try:
                if isinstance(item, dict):
                    positions.append(float(item["pos_m"]))
                else:
                    positions.append(float(item))
            except (KeyError, TypeError, ValueError):
                continue
        return sorted(set(positions))

    def _previous_balise_pos(self, pos_m: float) -> float:
        previous = [item for item in self.balises if item <= pos_m]
        return previous[-1] if previous else pos_m

    def _next_balise_pos(self, pos_m: float) -> float:
        for item in self.balises:
            if item > pos_m:
                return item
        return pos_m + self.effective_balise_spacing_m()

    def receive_safe_packet(self, packet: SafeMovementPacket, arrival_time_s: float):
        self.cc.receive_safe_packet(packet, arrival_time_s)

    def receive_vital_packet(self, packet: object, arrival_time_s: float):
        self.cc.receive_vital_packet(packet, arrival_time_s)

    def _refresh_controller_health(self):
        was_ato_fault_active = self.ato_fault_active
        self.cc_a.refresh()
        self.cc_b.refresh()
        self.ato_fault_active = not (self.cc_a.ato_ok or self.cc_b.ato_ok)
        self.atp_fault_active = not (self.cc_a.atp_ok or self.cc_b.atp_ok)
        if self.ato_fault_active:
            self.ato_recovery_state = "FAULT"
        elif was_ato_fault_active and self.ato_recovery_state == "FAULT":
            self.ato_recovery_state = "READY"

    def update_controller_redundancy(self, now_s: float):
        self._refresh_controller_health()
        previous = self.active_controller.name if self.active_controller is not None else "NONE"
        if self.active_controller is not None and self.active_controller.controller_ok:
            selected = self.active_controller
        elif self.standby_controller is not None and self.standby_controller.controller_ok:
            selected = self.standby_controller
        elif self.cc_a.controller_ok:
            selected = self.cc_a
        elif self.cc_b.controller_ok:
            selected = self.cc_b
        else:
            selected = None

        standby = None
        if selected is self.cc_a and self.cc_b.controller_ok:
            standby = self.cc_b
        elif selected is self.cc_b and self.cc_a.controller_ok:
            standby = self.cc_a

        self.active_controller = selected
        self.standby_controller = standby
        for channel in (self.cc_a, self.cc_b):
            channel.active = channel is selected
            channel.standby = channel is standby
            channel.last_switch_time = self.last_controller_switch_time

        current = selected.name if selected is not None else "NONE"
        if current != previous:
            self.controller_switch_count += 1
            self.last_controller_switch_time = now_s
            for channel in (self.cc_a, self.cc_b):
                channel.last_switch_time = now_s
            self.log_event("CONTROLLER_SWITCH", f"{previous}->{current}")

    def _set_controller_fault(self, fault_type: str, active: bool, now_s: float):
        token = fault_type.upper().removesuffix("_FAULT")
        if token in {"ATO", "ATP"}:
            targets = (self.cc_a, self.cc_b)
            subsystem = token
        else:
            parts = token.split("_")
            if len(parts) < 3 or parts[0] not in {"ATO", "ATP"}:
                return False
            subsystem = parts[0]
            target_name = "_".join(parts[1:])
            targets = (self.cc_a, self.cc_b) if target_name == "BOTH" else tuple(
                channel for channel in (self.cc_a, self.cc_b) if channel.name == target_name
            )
        for channel in targets:
            if subsystem == "ATO":
                channel.ato_ok = not active
            elif subsystem == "ATP":
                channel.atp_ok = not active
        self.update_controller_redundancy(now_s)
        return bool(targets)

    def controller_status_payload(self) -> Dict[str, Any]:
        active_name = self.active_controller.name if self.active_controller is not None else "NONE"
        standby_name = self.standby_controller.name if self.standby_controller is not None else "NONE"
        fault_reason = " | ".join(
            item
            for item in (
                f"CC_A: {self.cc_a.fault_reason}" if self.cc_a.fault_reason else "",
                f"CC_B: {self.cc_b.fault_reason}" if self.cc_b.fault_reason else "",
            )
            if item
        )
        return {
            "active_controller": active_name,
            "standby_controller": standby_name,
            "cc_a_status": self.cc_a.status,
            "cc_b_status": self.cc_b.status,
            "cc_a_fault_reason": self.cc_a.fault_reason,
            "cc_b_fault_reason": self.cc_b.fault_reason,
            "fault_reason": fault_reason,
            "controller_switch_count": int(self.controller_switch_count),
            "switch_counter": int(self.controller_switch_count),
            "last_switch_time": float(self.last_controller_switch_time),
            "ato_recovery_state": self.ato_recovery_state,
            "cc_a_active": bool(self.cc_a.active),
            "cc_b_active": bool(self.cc_b.active),
            "cc_a_standby": bool(self.cc_a.standby),
            "cc_b_standby": bool(self.cc_b.standby),
        }

    def apply_ato_unavailable(self):
        self.drive_mode = "LMD"
        self.mode_transition_reason = "ATO Fault - Manual Driving Required"
        self.ato_state = "ATO_UNAVAILABLE"
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.jog_active = False

    def confirm_ato_ready(self):
        if self.ato_recovery_state == "READY" and not self.ato_fault_active:
            self.ato_recovery_state = "CONFIRMED"
            self.ato_state = "ATO_CONFIRMED"
            self.mode_transition_reason = "ATO ready confirmed - start required"

    def start_ato_after_ready(self):
        if self.ato_recovery_state == "CONFIRMED" and not self.ato_fault_active and self.safe_packet_valid:
            self.ato_recovery_state = "NORMAL"
            self.drive_mode = "ATO"
            self.requested_drive_mode = "ATO"
            self.mode_transition_reason = ""
            self.ato_state = "ATO_STANDBY"

    def set_fault(self, subsystem: str, active: bool, now_s: float = 0.0):
        subsystem = subsystem.upper()
        if subsystem == "DCS":
            self.dcs_fault_active = bool(active)
            if active:
                self.dcs_mute_windows.append({"start_s": now_s, "end_s": now_s + 3600.0})
                self.dcs_muted = True
                self.log_event("DCS_LOSS_ACTIVE", "fault_injected")
            else:
                self.dcs_mute_windows = []
                self.dcs_muted = False
                self.cc.mark_received(now_s)
                self.safe_packet_valid = True
                self.dcs_degraded_requested = False
        elif subsystem == "ATO":
            self._set_controller_fault("ATO_BOTH_FAULT", active, now_s)
            if self.ato_fault_active:
                self.apply_ato_unavailable()
            elif self.ato_recovery_state == "READY":
                self.drive_mode = "LMD"
                self.ato_state = "ATO_READY"
                self.mode_transition_reason = "ATO ready - confirm required"
        elif subsystem == "ATP":
            self._set_controller_fault("ATP_BOTH_FAULT", active, now_s)
            if self.atp_fault_active:
                self.enter_trip_mode("ATP FAULT", self.reported_pos)
                self.atp_state = "ATP_TRIP"
                self.atp_alert = "ATP FAULT"
                self.atp_action = "EBI"
                self.atp_brake = "EMERGENCY"
                self.emg_latch = True
                self.log_event("ATP_FAULT_FAIL_SAFE_TRIP", "fault_injected")
            else:
                if self.trip_mode and self.trip_reason == "ATP FAULT":
                    self.atp_alert = "FAULT CLEARED - CONFIRM SAFE"
                self.log_event("ATP_FAULT_CLEARED", "awaiting_safe_recovery")
        elif subsystem in {
            "ATO_CC_A_FAULT",
            "ATO_CC_B_FAULT",
            "ATO_BOTH_FAULT",
            "ATP_CC_A_FAULT",
            "ATP_CC_B_FAULT",
            "ATP_BOTH_FAULT",
        }:
            self._set_controller_fault(subsystem, active, now_s)
            if self.ato_fault_active:
                self.apply_ato_unavailable()
            elif self.ato_recovery_state == "READY":
                self.drive_mode = "LMD"
                self.ato_state = "ATO_READY"
                self.mode_transition_reason = "ATO ready - confirm required"
            if self.atp_fault_active:
                self.enter_trip_mode("ATP FAULT", self.reported_pos)
                self.atp_state = "ATP_TRIP"
                self.atp_alert = "ATP FAULT"
                self.atp_action = "EBI"
                self.atp_brake = "EMERGENCY"
                self.emg_latch = True
                self.log_event("ATP_FAULT_FAIL_SAFE_TRIP", "both_controllers_failed")
            elif subsystem.startswith("ATP_") and not active and self.trip_mode and self.trip_reason == "ATP FAULT":
                self.atp_alert = "FAULT CLEARED - CONFIRM SAFE"
        elif subsystem in {"INTEGRITY", "COUPLER", "COUPLER_BREAK", "CONSIST_BREAK"}:
            self.integrity_fault_active = bool(active)
            if active:
                if self.virtual_integrity_links:
                    self.virtual_integrity_links[0] = False
                self.enter_trip_mode("TRAIN INTEGRITY LOST", self.reported_pos)
                self.atp_state = "ATP_TRIP"
                self.atp_alert = "TRAIN INTEGRITY LOST"
                self.atp_action = "EBI"
                self.atp_brake = "EMERGENCY"
                self.emg_latch = True
                self.log_event("TRAIN_INTEGRITY_LOST", "virtual_integrity_line_open")
            else:
                self.virtual_integrity_links = [True for _ in range(max(0, self.car_count - 1))]
                if self.trip_mode and self.trip_reason == "TRAIN INTEGRITY LOST":
                    self.atp_alert = "INTEGRITY RESTORED - CONFIRM SAFE"
                self.log_event("TRAIN_INTEGRITY_RESTORED", "awaiting_safe_recovery")

    def compute_ato_pid_accel(self, error: float, control_speed: float, a_service: float, a_traction: float) -> float:
        if abs(error) < kmh_to_ms(0.2):
            error = 0.0
        if error == 0.0:
            self.ato_pid_integral *= 0.6
        else:
            self.ato_pid_integral += error * DT
            self.ato_pid_integral = max(-ATO_PID_INT_LIMIT_MS, min(ATO_PID_INT_LIMIT_MS, self.ato_pid_integral))
        derivative = (error - self.ato_pid_prev_error) / max(DT, 1e-6)
        self.ato_pid_prev_error = error
        kp, ki, kd = ato_pid_gains(control_speed)
        pid_accel = kp * error + ki * self.ato_pid_integral + kd * derivative
        cmd_with_grade = pid_accel + G * self.gradient
        unsaturated_cmd = cmd_with_grade
        if cmd_with_grade > 0.0:
            cmd_with_grade = min(a_traction, cmd_with_grade)
        else:
            cmd_with_grade = max(-a_service, cmd_with_grade)
        saturated_high = unsaturated_cmd > a_traction and error > 0.0
        saturated_low = unsaturated_cmd < -a_service and error < 0.0
        if saturated_high or saturated_low:
            self.ato_pid_integral *= 0.85
        return cmd_with_grade

    def enter_trip_mode(self, reason: str, protect_pos: float | None = None):
        if protect_pos is None:
            protect_pos = self.reported_pos
        self.trip_mode = True
        self.trip_reason = reason
        self.trip_protect_pos = min(self.eoa, protect_pos)
        self.trip_protect_rear_pos = min(self.safe_rear_end_pos(), self.trip_protect_pos - self.length)
        self.emergency_stop = True
        self.emg_latch = True
        self.emg_ack = False
        self.release_active = False
        self.service_brake_latch = False
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0

    def clear_trip_mode(self):
        self.trip_mode = False
        self.trip_reason = ""
        self.emergency_stop = False
        self.trip_protect_rear_pos = self.safe_rear_end_pos()

    def acknowledge_emergency_safe(self):
        if self.speed > 0.01:
            return
        self.emg_latch = False
        self.emg_ack = False
        self.emergency_stop = False
        self.service_brake_latch = False
        self.clear_trip_mode()
        self.emergency_recovery_hold = True
        self.standstill_required = True
        self.standstill_anchor_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0

    def reset_non_emergency_stop_latches(self):
        self.service_brake_latch = False
        if not self.emg_latch and not self.trip_mode:
            self.emergency_stop = False
            self.emergency_recovery_hold = False
            self.clear_trip_mode()

    def apply_display_curve_smoothing(self, raw_curves: Dict[str, float], raw_hidden_curves: Dict[str, float]):
        if not self._display_curves_initialized:
            self.curves = dict(raw_curves)
            self.hidden_curves = dict(raw_hidden_curves)
            self._display_curves_initialized = True
            return

        precise_stop_display = (
            self.commanded_stop
            and not self.trip_mode
            and (
                0.0 <= self.distance_to_stop_target() <= PRECISE_STOP_SERVICE_BAND_M
                or (
                    0.0 <= self.distance_to_stop_target() <= JOG_MAX_DIST_M
                    and (self.zero_speed_detected or self.jog_state == JOG_STATE_ACTIVE)
                )
            )
        )
        if precise_stop_display:
            self.curves = dict(raw_curves)
            self.hidden_curves = dict(raw_hidden_curves)
            return

        previous_curves = self._previous_display_curves_for_smoothing or self.curves
        previous_hidden = self._previous_display_hidden_for_smoothing or self.hidden_curves
        previous = {
            "P": previous_curves.get("P", 0.0),
            "W": previous_curves.get("W", 0.0),
            "SBD": previous_curves.get("SBD", 0.0),
            "EBD": previous_curves.get("EBD", 0.0),
            "I": previous_hidden.get("I", 0.0),
            "OFF": previous_hidden.get("OFF", 0.0),
            "SBI": previous_hidden.get("SBI", 0.0),
            "EBI": previous_hidden.get("EBI", 0.0),
        }
        raw_all = {
            "P": raw_curves.get("P", 0.0),
            "W": raw_curves.get("W", 0.0),
            "SBD": raw_curves.get("SBD", 0.0),
            "EBD": raw_curves.get("EBD", 0.0),
            "I": raw_hidden_curves.get("I", 0.0),
            "OFF": raw_hidden_curves.get("OFF", 0.0),
            "SBI": raw_hidden_curves.get("SBI", 0.0),
            "EBI": raw_hidden_curves.get("EBI", 0.0),
        }

        smoothed: Dict[str, float] = {}
        bypass_all = self.trip_mode or self.emergency_stop
        for key, raw_value in raw_all.items():
            prev_value = previous.get(key, raw_value)
            if bypass_all or raw_value <= 0.0 or prev_value <= 0.0:
                smoothed[key] = raw_value
                continue
            if raw_value >= prev_value:
                max_rise = kmh_to_ms(CURVE_DISPLAY_RISE_RATE_KMH_S.get(key, 24.0)) * DT
                smoothed[key] = min(raw_value, prev_value + max_rise)
                continue

            safety_gap_kmh = 1.0 if key in ("EBI", "EBD") else 0.5
            raw_too_close_to_train = raw_value <= self.vital_speed + kmh_to_ms(safety_gap_kmh)
            if raw_too_close_to_train:
                smoothed[key] = raw_value
                continue

            max_drop = kmh_to_ms(CURVE_DISPLAY_DROP_RATE_KMH_S.get(key, 16.0)) * DT
            smoothed[key] = max(raw_value, prev_value - max_drop)

        def keep_below_display(value: float, upper: float) -> float:
            if upper <= 0.0:
                return 0.0
            if value >= upper:
                return max(0.0, upper - kmh_to_ms(CURVE_EPS_KMH))
            return value

        smoothed["EBI"] = keep_below_display(smoothed["EBI"], smoothed["EBD"])
        smoothed["SBD"] = keep_below_display(smoothed["SBD"], smoothed["EBI"])
        smoothed["SBI"] = keep_below_display(smoothed["SBI"], smoothed["SBD"])
        smoothed["OFF"] = keep_below_display(smoothed["OFF"], smoothed["SBI"])
        smoothed["W"] = keep_below_display(smoothed["W"], smoothed["OFF"])
        smoothed["P"] = keep_below_display(smoothed["P"], smoothed["W"])
        smoothed["I"] = min(smoothed["I"], max(0.0, smoothed["P"] - kmh_to_ms(CURVE_EPS_KMH)))

        self.curves = {"P": smoothed["P"], "W": smoothed["W"], "SBD": smoothed["SBD"], "EBD": smoothed["EBD"]}
        self.hidden_curves = {
            "I": smoothed["I"],
            "OFF": smoothed["OFF"],
            "SBI": smoothed["SBI"],
            "EBI": smoothed["EBI"],
        }

    def resume_after_emergency(self):
        if not self.emergency_recovery_hold:
            return
        self.emergency_recovery_hold = False
        self.emg_ack = False
        self.emergency_stop = False
        self.service_brake_latch = False
        self.clear_trip_mode()
        self.standstill_required = False
        self.standstill_anchor_pos = self.pos
        self.prev_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0
        if (
            self.requested_drive_mode == "ATO"
            and self.safe_packet_valid
            and not self.dcs_degraded_requested
            and not self.ato_fault_active
            and self.ato_recovery_state == "NORMAL"
        ):
            self.drive_mode = "ATO"
            self.mode_transition_reason = ""

    def distance_to_active_stop(self) -> float:
        if self.active_scheduled_stop is None:
            return float("inf")
        return float(self.active_scheduled_stop["pos_m"]) - self.pos

    def in_station_calibration_zone(self) -> bool:
        return self.commanded_stop and 0.0 <= self.distance_to_active_stop() <= STATION_BALISE_ZONE_M

    def effective_position_uncertainty_m(self) -> float:
        if self.beacon_position_locked:
            return PRECISE_STOP_POS_UNCERT_M
        if self.in_station_calibration_zone():
            return STATION_POS_UNCERT_M
        if self.has_balise_fix:
            return BALISE_POS_UNCERT_M
        return POS_UNCERT_M

    def train_integrity_ok(self) -> bool:
        return bool(not self.integrity_fault_active and all(self.virtual_integrity_links))

    def safe_rear_end_pos(self) -> float:
        """Conservative rear-end location used by ZC for moving-block MAL."""
        return self.reported_pos - self.length - self.effective_position_uncertainty_m()

    def effective_balise_spacing_m(self) -> float:
        if self.in_station_calibration_zone():
            return STATION_BALISE_SPACING_M
        return BALISE_SPACING_M

    def effective_balise_error_max_m(self) -> float:
        if self.beacon_position_locked:
            return 0.0
        if self.in_station_calibration_zone():
            return STATION_BALISE_ERROR_MAX_M
        return BALISE_ERROR_MAX_M

    def sync_reported_position(self):
        self.reported_pos = self.pos
        self.pos_error_m = 0.0

    def distance_to_stop_target(self) -> float:
        return self.stop_target_pos - self.pos

    def scheduled_stop_target_pos(self) -> float | None:
        if self.active_scheduled_stop is None:
            return None
        return float(self.active_scheduled_stop["pos_m"])

    def resolve_stop_target(self, authority_target_pos: float) -> float:
        station_target_pos = self.scheduled_stop_target_pos()
        if (
            self.active_scheduled_stop is not None
            and self.station_state in {
                "APPROACHING_STATION",
                "ROUTE_ASSIGNED",
                "DOCKING",
                "STOPPED_AT_PLATFORM",
                "DWELLING",
                "READY_TO_DEPART",
            }
            and station_target_pos is not None
            and authority_target_pos + JOG_WINDOW_EPS_M >= station_target_pos
        ):
            # For a station stop, keep the platform marker as the commanded target
            # while authority still covers it. EOA/SvL continues to protect ATP.
            return station_target_pos
        return authority_target_pos

    def stop_identity(self) -> str:
        if self.active_scheduled_stop is not None:
            name = self.active_scheduled_stop.get("name", "STOP")
            pos_m = float(self.active_scheduled_stop["pos_m"])
            return f"{name}@{pos_m:.3f}"
        return f"target@{self.stop_target_pos:.3f}"

    def get_current_stop_key(self) -> str | None:
        if not self.commanded_stop:
            return None
        return self.stop_identity()

    def jog_limit_m(self) -> float:
        return min(JOG_MAX_DIST_M, MANUAL_JOG_WINDOW_M)

    def docking_jog_limit_m(self) -> float:
        return min(JOG_MAX_DIST_M, max(MANUAL_JOG_WINDOW_M, AUTO_DOCKING_JOG_WINDOW_M))

    def _curve_snapshot(self) -> Dict[str, float]:
        return {
            "P": ms_to_kmh(self.curves.get("P", 0.0)),
            "W": ms_to_kmh(self.curves.get("W", 0.0)),
            "SBI": ms_to_kmh(self.hidden_curves.get("SBI", 0.0)),
            "EBI": ms_to_kmh(self.hidden_curves.get("EBI", 0.0)),
            "EBD": ms_to_kmh(self.curves.get("EBD", 0.0)),
        }

    def log_event(self, event: str, reason: str = "", **extra):
        stop_error_m = self.pos - self.stop_target_pos
        record = {
            "sim_time": self.last_sim_time_s,
            "train_id": self.id,
            "event": event,
            "reason": reason,
            "stop_id": self.current_stop_id or self.stop_identity(),
            "current_pos": self.pos,
            "stop_target_pos": self.stop_target_pos,
            "remaining": self.stop_target_pos - self.pos,
            "stop_error": stop_error_m,
            "speed_kmh": ms_to_kmh(self.speed),
            "vital_speed_kmh": ms_to_kmh(self.vital_speed),
            "eoa_m": self.eoa,
            "distance_to_eoa": self.distance_to_eoa,
            "station_state": self.station_state,
            "assigned_line": self.assigned_platform,
            "dwell_remaining": self.dwell_remaining_s,
            "curve_mode": self.curve_mode,
            "jog_state": self.jog_state,
            "curves": self._curve_snapshot(),
        }
        record.update(extra)
        self.event_records.append(record)
        self.pending_event_records.append(record)

    def pop_pending_events(self):
        events = list(self.pending_event_records)
        self.pending_event_records.clear()
        return events

    def _set_jog_state(self, state: str, reason: str = ""):
        if self.jog_state == state and not reason:
            return
        self.jog_state = state
        self.precise_jog_in_progress = state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE)
        self.precise_jog_completed = state == JOG_STATE_COMPLETED
        self.jog_active = state == JOG_STATE_ACTIVE
        if state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE, JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED):
            self.jog_used = True
            self.jog_used_for_current_stop = True
            self.jog_used_stop_key = self.active_stop_key or self.current_stop_id or self.stop_identity()
        event_by_state = {
            JOG_STATE_REQUESTED: "JOG_REQUESTED",
            JOG_STATE_ACTIVE: "JOG_STARTED",
            JOG_STATE_COMPLETED: "JOG_COMPLETED",
            JOG_STATE_FAILED_LOCKED: "JOG_FAILED_LOCKED",
        }
        event = event_by_state.get(state)
        if event:
            self.log_event(event, reason)
            if event == "JOG_STARTED":
                key = self.current_stop_id or self.stop_identity()
                self.jog_event_counts[key] = self.jog_event_counts.get(key, 0) + 1

    def _reset_jog_for_stop(self, stop_id: str | None):
        self.current_stop_id = stop_id
        self.active_stop_key = stop_id
        self.jog_state = JOG_STATE_IDLE
        self.jog_used = False
        self.jog_used_for_current_stop = False
        self.jog_used_stop_key = None
        self.manual_jog_requested = False
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_active = False
        self.jog_stop_target_pos = None
        self.jog_start_pos = self.pos
        self.jog_profile_distance_m = 0.0

    def can_request_precise_jog(self) -> bool:
        key = self.get_current_stop_key()
        remaining_m = self.distance_to_stop_target()
        eoa_error_m = abs(self.eoa - self.pos)
        jog_limit = self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
        station_target_pos = self.scheduled_stop_target_pos()
        if station_target_pos is not None and self.stop_target_pos < station_target_pos - JOG_WINDOW_EPS_M:
            return False
        return (
            key is not None
            and self.commanded_stop
            and (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS)
            and not self.door_authorized
            and not self.jog_active
            and not self.jog_used
            and self.jog_state == JOG_STATE_IDLE
            and not self.jog_used_for_current_stop
            and self.jog_used_stop_key != key
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
            and STOP_ACCURACY_TOL_M < remaining_m <= jog_limit
            and eoa_error_m <= jog_limit
        )

    def request_precise_jog(self) -> bool:
        stop_id = self.get_current_stop_key()
        if stop_id is None:
            self.log_event("JOG_IGNORED_NO_STOP")
            return False
        if self.jog_used_stop_key == stop_id or self.jog_used_for_current_stop:
            self.log_event("JOG_IGNORED_ALREADY_USED", f"state={self.jog_state}")
            return False
        if self.jog_state != JOG_STATE_IDLE:
            self.log_event("JOG_IGNORED_LOCKED", f"state={self.jog_state}")
            return False
        if not (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS):
            self.log_event("JOG_IGNORED_NOT_ZERO_SPEED")
            return False
        if self.door_authorized:
            self.log_event("JOG_IGNORED_DOOR_ALREADY_AUTHORIZED")
            return False
        if self.emergency_stop or self.emg_latch or self.trip_mode or self.emergency_recovery_hold:
            self.log_event("JOG_IGNORED_ATP_TRIP")
            return False
        remaining_m = self.distance_to_stop_target()
        if remaining_m <= 0.0:
            self.current_stop_id = stop_id
            self.active_stop_key = stop_id
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, f"remaining={remaining_m:.3f}")
            self.log_event("JOG_IGNORED_OVERRUN", f"remaining={remaining_m:.3f}")
            return False
        if remaining_m <= STOP_ACCURACY_TOL_M:
            self.log_event("JOG_IGNORED_ALREADY_ALIGNED", f"remaining={remaining_m:.3f}")
            return False
        if remaining_m > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M:
            self.log_event("JOG_IGNORED_TOO_FAR", f"remaining={remaining_m:.3f}")
            return False
        if abs(self.eoa - self.pos) > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M or not self.can_request_precise_jog():
            self.log_event("JOG_IGNORED_INVALID_REMAINING", "preconditions")
            return False
        self.current_stop_id = stop_id
        self.active_stop_key = stop_id
        self.manual_jog_requested = False
        self.jog_used = True
        self.jog_used_for_current_stop = True
        self.jog_used_stop_key = stop_id
        self.jog_stop_target_pos = self.stop_target_pos
        self.jog_start_pos = self.pos
        self.jog_profile_distance_m = remaining_m
        self.standstill_required = False
        self.standstill_anchor_pos = self.pos
        self.ato_target_speed = 0.0
        self.ato_pid_integral = 0.0
        self.ato_pid_prev_error = 0.0
        self.prev_accel = 0.0
        self._set_jog_state(JOG_STATE_ACTIVE)
        return True

    def retry_failed_jog_if_safe(self) -> bool:
        if self.jog_state != JOG_STATE_FAILED_LOCKED:
            return False
        key = self.get_current_stop_key()
        remaining_m = self.distance_to_stop_target()
        jog_limit = self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
        if not (
            key is not None
            and self.commanded_stop
            and (self.zero_speed_detected or self.speed <= STANDSTILL_SPEED_EPS)
            and not self.door_authorized
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
            and STOP_ACCURACY_TOL_M < remaining_m <= jog_limit
            and abs(self.eoa - self.pos) <= jog_limit
        ):
            return False
        self.log_event("JOG_RETRY_SAFE", f"remaining={remaining_m:.3f}")
        self.jog_state = JOG_STATE_IDLE
        self.jog_used = False
        self.jog_used_for_current_stop = False
        self.jog_used_stop_key = None
        self.precise_jog_in_progress = False
        self.precise_jog_completed = False
        self.jog_active = False
        return True

    def _update_door_authorization(self):
        brake_applied_for_hold = self.standstill_required or self.service_brake_latch or self.emg_latch or self.commanded_stop
        station_target_pos = self.scheduled_stop_target_pos()
        station_stop_ready = (
            station_target_pos is None
            or (
                self.station_lane is not None
                and self.stop_target_pos >= station_target_pos - JOG_WINDOW_EPS_M
                and abs(self.pos - station_target_pos) <= STOP_ACCURACY_TOL_M
            )
        )
        self.door_authorized = (
            self.zero_speed_detected
            and brake_applied_for_hold
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and station_stop_ready
            and abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M
        )
        self.door_open_allowed = self.door_authorized
        if self.door_authorized and not self._last_door_authorized and self.jog_state == JOG_STATE_COMPLETED:
            self.log_event("DOOR_AUTHORIZED_AFTER_JOG")
        self._last_door_authorized = self.door_authorized

    def _finalize_jog_step(self):
        self.zero_speed_detected = self.speed <= STANDSTILL_SPEED_EPS
        self.standstill_monitoring = self.standstill_required or self.zero_speed_detected
        self.rollback_monitoring = self.zero_speed_detected or self.commanded_stop or self.emg_latch
        self._update_door_authorization()
        self.ato_hold_active = self.zero_speed_detected and (
            self.standstill_required or self.ato_target_speed <= STANDSTILL_SPEED_EPS
        )
        if self.door_authorized:
            self.ato_door_mode = "ENABLE"
        elif self.ato_hold_active:
            self.ato_door_mode = "READY"
        else:
            self.ato_door_mode = "LOCKED"
        if self.jog_state == JOG_STATE_ACTIVE:
            self.precise_stop_state = "JOG"
            self.ato_state = "ATO_JOG"
        elif self.jog_state == JOG_STATE_FAILED_LOCKED:
            self.precise_stop_state = "JOG_FAILED"
            self.ato_state = "ATO_HOLD"
        elif abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M:
            self.precise_stop_state = "ALIGNED"
            self.ato_state = "ATO_HOLD"
        else:
            self.precise_stop_state = "WAIT_JOG"
            self.ato_state = "ATO_HOLD"
        if self.trip_mode:
            self.atp_state = "ATP_TRIP"
            self.atp_alert = self.trip_reason or "TRAIN TRIP"
            self.atp_action = "EBI"
        elif self.emg_latch or self.emergency_stop:
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "JOG PROTECTION"
            self.atp_action = "EBI"
        elif self.door_authorized:
            self.atp_state = "ATP_STANDSTILL"
            self.atp_alert = "DOOR ENABLE"
            self.atp_action = ""
        else:
            self.atp_state = "ATP_STANDSTILL" if self.zero_speed_detected else "ATP_OK"
            self.atp_alert = "STANDSTILL MON" if self.zero_speed_detected else "OK"
            self.atp_action = ""
        self.prev_pos = self.pos
        self.prev_commanded_stop = self.commanded_stop

    def update_jog(self, dt_s: float) -> bool:
        if self.jog_state != JOG_STATE_ACTIVE:
            return False
        key = self.get_current_stop_key()
        target = self.stop_target_pos
        remaining_m = target - self.pos
        if key is None:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "no_stop")
            self.log_event("JOG_FAILED_LOCKED", "no_stop")
            self._finalize_jog_step()
            return True
        if self.emergency_stop or self.emg_latch or self.trip_mode:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "atp_trip")
            self.log_event("JOG_FAILED_LOCKED", "atp_trip")
            self._finalize_jog_step()
            return True
        if remaining_m <= STOP_ACCURACY_TOL_M and remaining_m >= 0.0:
            self.pos = target
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "within_tolerance")
            if abs(self.pos - target) > STOP_ACCURACY_TOL_M:
                self.log_event("JOG_COMPLETED_NOT_ALIGNED")
            self._finalize_jog_step()
            return True
        if remaining_m < 0.0:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "overrun")
            self.log_event("JOG_FAILED_OVERRUN", f"remaining={remaining_m:.3f}")
            self._finalize_jog_step()
            return True
        if remaining_m > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M:
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "too_far")
            self.log_event("JOG_FAILED_TRUE_TOO_FAR", f"remaining={remaining_m:.3f}")
            self._finalize_jog_step()
            return True
        total_m = max(self.jog_profile_distance_m, target - self.jog_start_pos, remaining_m)
        travelled_m = max(0.0, self.pos - self.jog_start_pos)
        half_m = max(0.001, 0.5 * total_m)
        configured_jog_speed_ms = kmh_to_ms(JOG_SPEED_KMH)
        distance_limited_speed_ms = (2.0 * JOG_PROFILE_ACCEL_MS2 * half_m) ** 0.5
        max_profile_speed_ms = min(configured_jog_speed_ms, distance_limited_speed_ms)
        speed_limit_source = "configured_jog_speed" if configured_jog_speed_ms <= distance_limited_speed_ms else "distance_accel_profile"
        if travelled_m < half_m:
            commanded_accel = JOG_PROFILE_ACCEL_MS2
            profile_phase = "ACCEL"
        else:
            commanded_accel = -JOG_PROFILE_ACCEL_MS2
            profile_phase = "BRAKE"
        next_speed = max(0.0, min(max_profile_speed_ms, self.speed + commanded_accel * dt_s))
        target_speed_reason = speed_limit_source
        if travelled_m >= half_m and remaining_m <= STOP_ACCURACY_TOL_M + max(0.0, self.speed * dt_s):
            next_speed = 0.0
            target_speed_reason = "final_tolerance_stop"
        self.log_event(
            "JOG_PROFILE_TRACE",
            target_speed_reason,
            configured_jog_speed_kmh=JOG_SPEED_KMH,
            jog_target_speed_kmh=ms_to_kmh(max_profile_speed_ms),
            distance_limited_speed_kmh=ms_to_kmh(distance_limited_speed_ms),
            actual_speed_kmh=ms_to_kmh(self.speed),
            next_speed_kmh=ms_to_kmh(next_speed),
            commanded_accel_ms2=commanded_accel,
            profile_phase=profile_phase,
            travelled_m=travelled_m,
            profile_distance_m=total_m,
            half_distance_m=half_m,
            atp_p_kmh=ms_to_kmh(self.curves.get("P", 0.0)),
            atp_w_kmh=ms_to_kmh(self.curves.get("W", 0.0)),
            atp_sbi_kmh=ms_to_kmh(self.hidden_curves.get("SBI", 0.0)),
            atp_ebi_kmh=ms_to_kmh(self.hidden_curves.get("EBI", 0.0)),
            selected_reference="jog_motion_profile",
        )
        step_m = 0.5 * (self.speed + next_speed) * dt_s
        if step_m >= remaining_m or next_speed <= STANDSTILL_SPEED_EPS and travelled_m >= half_m:
            self.pos = target
            self.speed = 0.0
            self.prev_accel = 0.0
            self.ato_target_speed = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "clamped_at_target")
            self.log_event("JOG_COMPLETED_CLAMPED_TO_TARGET")
            self._finalize_jog_step()
            return True
        self.pos += step_m
        self.reported_pos = self.pos
        self.pos_error_m = 0.0
        self.speed = next_speed
        self.prev_accel = commanded_accel
        self.ato_target_speed = 0.0
        self.standstill_required = False
        self.jog_active = True
        self.log_event("JOG_CREEP_ACTIVE", f"remaining={remaining_m:.3f}")
        self.log_event("JOG_ACTIVE_STEP")
        self._finalize_jog_step()
        return True

    def update_reported_position(self):
        error_cap_m = self.effective_balise_error_max_m()
        if self.pos >= self.next_balise_pos:
            self.last_balise_pos = self.next_balise_pos
            self.next_balise_pos = self._next_balise_pos(self.next_balise_pos)
            self.has_balise_fix = True
            self.pos_error_m = 0.0
            self.odometer_error_sign *= -1.0

        if self.beacon_position_locked:
            self.sync_reported_position()
            return

        dist_since = max(0.0, self.pos - self.last_balise_pos)
        error = min(error_cap_m, ODOMETER_ERROR_RATE * dist_since)
        self.pos_error_m = self.odometer_error_sign * error
        self.reported_pos = self.pos + self.pos_error_m

    def step(self, now_s: float):
        self.last_sim_time_s = now_s
        self.update_controller_redundancy(now_s)
        self.cc.apply_to_train(self, now_s)
        station_stop_eoa = None
        if self.commanded_stop and self.active_scheduled_stop is not None and self.station_lane is not None:
            stop_pos = float(self.active_scheduled_stop["pos_m"])
            station_end = stop_pos + float(self.active_scheduled_stop.get("length_m", 160.0)) / 2.0
            if self.pos <= station_end:
                station_stop_eoa = stop_pos - (STOP_SVL_OFFSET_M - STOP_TARGET_OFFSET_M)
                self.eoa = min(self.eoa, station_stop_eoa)
                remaining_to_station_stop = stop_pos - self.pos
                final_station_capture = (
                    -STOP_ACCURACY_TOL_M <= remaining_to_station_stop <= self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
                    and self.speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH + 0.5)
                    and not self.trip_mode
                    and not self.emergency_stop
                    and not self.emg_latch
                )
                if final_station_capture:
                    self.eoa = max(self.eoa, station_stop_eoa)
        hard_departure_hold = self.departure_hold and self.protection_zone_id == "SOURCE"
        stop_short_pending = (
            self.commanded_stop
            and self.distance_to_stop_target() > STOP_ACCURACY_TOL_M
        )
        if hard_departure_hold and self.speed <= kmh_to_ms(1.0) and not stop_short_pending:
            self.speed = 0.0
            self.filtered_speed = 0.0
            self.prev_accel = 0.0
            if not self.standstill_required:
                self.standstill_anchor_pos = self.pos
            self.standstill_required = True
        elif (
            stop_short_pending
            and self.dwell_remaining_s <= 0.0
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
        ):
            self.standstill_required = False
        elif not (self.commanded_stop or self.emg_latch or self.trip_mode or self.emergency_recovery_hold):
            self.standstill_required = False
        self.filtered_speed = low_pass_step(self.filtered_speed, self.speed, SPEED_FILTER_TAU_S, DT)
        self.atp_brake = "NONE"
        self.atp_action = ""
        self.traction_cutoff = False
        if self.ato_fault_active:
            self.apply_ato_unavailable()
        elif self.ato_recovery_state == "READY":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_READY"
            self.mode_transition_reason = "ATO ready - confirm required"
        elif self.ato_recovery_state == "CONFIRMED":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_CONFIRMED"
            self.mode_transition_reason = "ATO ready confirmed - start required"
        if self.atp_fault_active:
            self.enter_trip_mode("ATP FAULT", self.reported_pos)
            self.atp_state = "ATP_TRIP"
            self.atp_alert = "ATP FAULT"
            self.atp_action = "EBI"
            self.atp_brake = "EMERGENCY"
            self.emg_latch = True
        if not self.train_integrity_ok():
            self.enter_trip_mode("TRAIN INTEGRITY LOST", self.reported_pos)
            self.atp_state = "ATP_TRIP"
            self.atp_alert = "TRAIN INTEGRITY LOST"
            self.atp_action = "EBI"
            self.atp_brake = "EMERGENCY"
            self.emg_latch = True
        if not self.safe_packet_valid and self.train_integrity_ok():
            if self.drive_mode == "ATO":
                self.drive_mode = "CMD25"
                self.dcs_degraded_requested = True
                self.mode_transition_reason = "DCS timeout: ATO inhibited, restricted recovery required"
            self.enter_trip_mode("DCS TIMEOUT", self.reported_pos)

        position_uncertainty_m = self.effective_position_uncertainty_m()
        safe_margin = max(position_uncertainty_m, abs(self.pos_error_m))
        self.safe_front_end_pos = self.reported_pos + safe_margin
        protected_eoa = self.eoa
        if self.trip_mode:
            protected_eoa = min(protected_eoa, self.trip_protect_pos)
        dwell_stop_hold = self.dwell_remaining_s > 0.0 and self.standstill_required
        if dwell_stop_hold:
            held_eoa = self.standstill_anchor_pos + STOP_TARGET_OFFSET_M - STOP_SVL_OFFSET_M
            protected_eoa = max(protected_eoa, held_eoa)
        target_eoa = protected_eoa - 1.0
        self.distance_to_eoa = target_eoa - self.safe_front_end_pos
        if self.distance_to_eoa < self.limit_ahead_dist:
            self.constraint_type = "STOP"
            self.constraint_target_speed_kmh = 0.0
            self.distance_to_constraint_m = self.distance_to_eoa
        elif self.limit_ahead_dist != float("inf"):
            self.constraint_type = "SPEED_REDUCTION"
            self.constraint_target_speed_kmh = self.limit_ahead_speed_kmh
            self.distance_to_constraint_m = self.limit_ahead_dist
        else:
            self.constraint_type = "NONE"
            self.constraint_target_speed_kmh = self.psr_kmh
            self.distance_to_constraint_m = float("inf")

        active_stop_eoa = protected_eoa if dwell_stop_hold else self.eoa
        svl_pos = active_stop_eoa + STOP_SVL_OFFSET_M
        distance_to_svl = svl_pos - self.reported_pos
        self.stop_target_pos = self.resolve_stop_target(svl_pos - STOP_TARGET_OFFSET_M)
        self.stop_beacon_pos = self.stop_target_pos - STOP_BEACON_OFFSET_M
        actual_distance_to_stop = self.stop_target_pos - self.pos
        self.release_active = False
        current_stop_id = self.stop_identity() if self.commanded_stop else None
        if current_stop_id is None:
            if self.current_stop_id is not None or self.jog_state != JOG_STATE_IDLE or self.jog_used_for_current_stop:
                self._reset_jog_for_stop(None)
        elif self.current_stop_id != current_stop_id:
            self._reset_jog_for_stop(current_stop_id)

        if self.pos >= self.stop_beacon_pos and not self.stop_beacon_seen:
            # Station stop beacon clears accumulated odometry error near the platform.
            self.sync_reported_position()
            self.has_balise_fix = True
            self.last_balise_pos = self.pos
            self.next_balise_pos = self._next_balise_pos(self.pos)
            self.stop_beacon_seen = True
            self.beacon_position_locked = True
            self.beacon_lock_stop_pos = self.stop_target_pos
        elif self.pos < self.stop_beacon_pos - 2.0:
            self.stop_beacon_seen = False
        if self.beacon_position_locked:
            locked_stop_pos = self.beacon_lock_stop_pos if self.beacon_lock_stop_pos is not None else self.stop_target_pos
            distance_from_locked_stop = self.pos - locked_stop_pos
            if distance_from_locked_stop < -STATION_BALISE_ZONE_M or distance_from_locked_stop > BEACON_LOCK_RELEASE_MARGIN_M:
                self.beacon_position_locked = False
                self.beacon_lock_stop_pos = None
            else:
                self.sync_reported_position()

        atp = self.atp_engine.compute(self)
        self.release_active = atp.release_active
        self.release_speed_kmh = atp.release_speed_kmh
        self.release_blend = atp.release_blend
        self.release_p_target_ms = atp.p_t
        self.release_p_release_ms = atp.p_r
        self._previous_display_curves_for_smoothing = dict(self.curves)
        self._previous_display_hidden_for_smoothing = dict(self.hidden_curves)
        self.raw_curves = dict(atp.curves)
        self.raw_hidden_curves = dict(atp.hidden_curves)
        self.apply_display_curve_smoothing(self.raw_curves, self.raw_hidden_curves)
        stop_ebi_floor_active = (
            self.commanded_stop
            and abs(atp.actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if stop_ebi_floor_active:
            self.hidden_curves["EBI"] = max(self.hidden_curves["EBI"], kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            self.curves["EBD"] = max(self.curves["EBD"], self.hidden_curves["EBI"])
            self.raw_hidden_curves["EBI"] = self.hidden_curves["EBI"]
            self.raw_curves["EBD"] = self.curves["EBD"]
        atp_supervision = replace(
            atp,
            curves=dict(self.curves),
            hidden_curves=dict(self.hidden_curves),
            cutoff_threshold=self.hidden_curves.get("OFF", atp.cutoff_threshold),
        )
        self.curve_mode = atp.curve_mode
        self.cutoff_threshold = atp_supervision.cutoff_threshold
        self.margin_dyn_m = atp.margin_dyn_m
        self.atp_service_brake_decel = atp.atp_service_brake_decel
        self.atp_emergency_brake_decel = atp.atp_emergency_brake_decel

        self.retry_failed_jog_if_safe()
        if self.can_request_precise_jog():
            self.request_precise_jog()

        if self.jog_state == JOG_STATE_ACTIVE and self.update_jog(DT):
            return

        ato = self.ato_engine.compute(self, atp_supervision)
        self.ato_brake_prepare = ato.ato_brake_prepare
        self.ato_curve_speed = ato.ato_curve_speed
        self.ato_piloting_speed = ato.ato_piloting_speed
        self.ato_target_speed = ato.ato_target_speed
        self.jog_active = ato.jog_active
        self.jog_used = ato.jog_used
        if self.jog_state == JOG_STATE_REQUESTED and self.jog_active:
            self._set_jog_state(JOG_STATE_ACTIVE)
        if self.jog_active or self.jog_used or not self.commanded_stop:
            self.manual_jog_requested = False
        if self.drive_mode == "LMD":
            manual_limit = min(kmh_to_ms(self.max_manual_speed_kmh), atp_supervision.curves["P"] - ato_tracking_margin_ms(atp_supervision.control_speed))
            self.ato_target_speed = max(0.0, manual_limit)
            self.jog_active = False
        elif self.drive_mode == "CMD25":
            degraded_limit = min(kmh_to_ms(25.0), atp_supervision.curves["P"] - ato_tracking_margin_ms(atp_supervision.control_speed))
            self.ato_target_speed = max(0.0, degraded_limit)
        if self.ato_fault_active:
            self.apply_ato_unavailable()
        elif self.ato_recovery_state == "READY":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_READY"
            self.ato_target_speed = 0.0
            self.jog_active = False
        elif self.ato_recovery_state == "CONFIRMED":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_CONFIRMED"
            self.ato_target_speed = 0.0
            self.jog_active = False
        if self.emergency_recovery_hold:
            self.ato_target_speed = 0.0
            self.jog_active = False
        if hard_departure_hold:
            self.ato_target_speed = 0.0
            self.jog_active = False
        if self.emergency_recovery_hold or self.emg_latch or self.trip_mode:
            if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, "interrupted_by_protection")
        if self.precise_jog_in_progress and self.jog_stop_target_pos is None:
            self.jog_stop_target_pos = self.stop_target_pos

        control_speed = atp_supervision.control_speed
        actual_distance_to_stop = atp_supervision.actual_distance_to_stop
        distance_to_svl = atp_supervision.distance_to_svl
        a_service = atp_supervision.a_service
        a_emergency = atp_supervision.a_emergency
        a_traction = atp_supervision.a_traction
        coast_decel = COAST_BASE_DECEL + COAST_SPEED_GAIN * control_speed

        stop_within_tolerance = abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
        stop_overshoot = actual_distance_to_stop < -STOP_ACCURACY_TOL_M
        if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
            if stop_within_tolerance:
                self.pos = self.stop_target_pos
                self.speed = 0.0
                self.ato_target_speed = 0.0
                self.prev_accel = 0.0
                self.standstill_required = True
                self.standstill_anchor_pos = self.pos
                self._set_jog_state(JOG_STATE_COMPLETED, "within_tolerance")
            elif actual_distance_to_stop <= 0.0:
                self.speed = 0.0
                self.ato_target_speed = 0.0
                self.prev_accel = 0.0
                reason = "overshot_target" if self.pos > self.stop_target_pos else "invalid_remaining"
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, reason)
            elif actual_distance_to_stop > JOG_MAX_DIST_M or not self.commanded_stop:
                self.speed = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else self.speed
                self.ato_target_speed = 0.0
                self._set_jog_state(JOG_STATE_FAILED_LOCKED, "invalid_remaining")
        # For commanded station stops, allow the train to finish braking with SBI while it is
        # still within the final stop tolerance. Only escalate to EB once it has genuinely
        # overshot the permitted stopping window.
        zero_speed_at_stop_limit = self.zero_speed_detected and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
        scheduled_stop_target = self.scheduled_stop_target_pos()
        temporary_authority_stop = (
            self.commanded_stop
            and scheduled_stop_target is not None
            and self.stop_target_pos < scheduled_stop_target - JOG_WINDOW_EPS_M
        )
        if temporary_authority_stop and self.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED):
            self._reset_jog_for_stop(self.current_stop_id or self.stop_identity())
        low_speed_temporary_hold = (
            temporary_authority_stop
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and distance_to_svl > 0.0
        )
        stop_position_emergency = self.distance_to_eoa <= 0.0 and (
            not zero_speed_at_stop_limit
            and not low_speed_temporary_hold
            and (not self.commanded_stop or stop_overshoot)
        )
        emergency = stop_position_emergency or self.emergency_stop
        downhill_buffer = kmh_to_ms(max(0.0, -self.gradient) * DOWNHILL_P_BUFFER_KMH_PER_GRAD)
        service_release_limit = max(0.0, self.curves["P"] - downhill_buffer - kmh_to_ms(SBI_RELEASE_HYST_KMH))
        service_speed_violation = self.vital_speed > self.hidden_curves["SBI"] + kmh_to_ms(SBI_VIOLATION_EPS_KMH)
        service_stop_distance = stopping_distance_with_buildup(
            self.vital_speed,
            max(a_service, ATP_MIN_DECEL_MS2),
            BRAKE_BUILDUP_S,
        )
        projected_service_stop_safe_before_svl = service_stop_distance <= max(0.0, distance_to_svl)
        station_jog_wait_guard = (
            self.commanded_stop
            and self.get_current_stop_key() is not None
            and self.jog_state == JOG_STATE_IDLE
            and self.jog_used_stop_key != self.get_current_stop_key()
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
            and self.vital_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and distance_to_svl > 0.0
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        low_speed_service_guard = (
            service_speed_violation
            and self.commanded_stop
            and 0.0 < actual_distance_to_stop <= RELEASE_HANDOVER_START_M
            and self.vital_speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH + 0.8)
            and distance_to_svl > 0.0
            and projected_service_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        precise_stop_service_intervention = (
            service_speed_violation
            and self.commanded_stop
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= PRECISE_STOP_SERVICE_BAND_M
            and self.vital_speed > STANDSTILL_SPEED_EPS
            and distance_to_svl > 0.0
            and service_stop_distance > max(STOP_ACCURACY_TOL_M, actual_distance_to_stop)
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if not self.commanded_stop:
            if (not self.release_active) and service_speed_violation:
                self.service_brake_latch = True
            elif self.service_brake_latch and self.vital_speed <= service_release_limit:
                self.service_brake_latch = False
        elif station_jog_wait_guard:
            if self.service_brake_latch:
                self.log_event("ATP_NUISANCE_GUARD_LOW_SPEED", "station_jog_wait_service_release")
            self.service_brake_latch = False
        elif self.release_active and self.vital_speed <= kmh_to_ms(RELEASE_SPEED_KMH):
            self.service_brake_latch = False
        elif (
            self.commanded_stop
            and self.service_brake_latch
            and self.vital_speed <= service_release_limit
            and not precise_stop_service_intervention
        ):
            self.service_brake_latch = False

        door_enable_service_guard = (
            service_speed_violation
            and self.door_authorized
            and self.zero_speed_detected
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and distance_to_svl > 0.0
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if door_enable_service_guard and self.service_brake_latch:
            self.service_brake_latch = False

        service_intervention = precise_stop_service_intervention or self.service_brake_latch or (
            service_speed_violation
            and not station_jog_wait_guard
            and not low_speed_service_guard
            and not door_enable_service_guard
        )

        stop_ebi_floor_active = (
            self.commanded_stop
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        ebi_limit_for_violation = self.hidden_curves["EBI"]
        ebd_limit_for_violation = self.curves["EBD"]
        if stop_ebi_floor_active:
            ebi_limit_for_violation = max(ebi_limit_for_violation, kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            ebd_limit_for_violation = max(ebd_limit_for_violation, ebi_limit_for_violation)
            self.hidden_curves["EBI"] = ebi_limit_for_violation
            self.raw_hidden_curves["EBI"] = ebi_limit_for_violation
        ebi_speed_violation = (
            self.vital_speed > ebi_limit_for_violation + 0.05
            or self.vital_speed > ebd_limit_for_violation + 0.05
        )
        if ebi_speed_violation:
            self._low_speed_ebi_counter += 1
        else:
            self._low_speed_ebi_counter = 0
            self._low_speed_guard_active = False
        emergency_stop_distance = stopping_distance_with_buildup(
            self.vital_speed,
            max(a_emergency, ATP_MIN_DECEL_MS2),
            BRAKE_BUILDUP_S,
        )
        projected_stop_safe_before_svl = emergency_stop_distance <= max(0.0, distance_to_svl)
        low_speed_station_guard = (
            ebi_speed_violation
            and self.commanded_stop
            and self.vital_speed <= kmh_to_ms(LOW_SPEED_ATP_GUARD_KMH)
            and -STOP_ACCURACY_TOL_M <= actual_distance_to_stop <= RELEASE_ZONE_M
            and distance_to_svl > 0.0
            and projected_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
        )
        low_speed_departure_hold_guard = (
            ebi_speed_violation
            and self.departure_hold
            and not hard_departure_hold
            and self.station_state in {"READY_TO_DEPART", "DEPARTING"}
            and self.vital_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and distance_to_svl > 0.0
            and projected_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        precise_stop_service_guard = (
            ebi_speed_violation
            and service_intervention
            and self.commanded_stop
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= PRECISE_STOP_SERVICE_BAND_M
            and distance_to_svl > 0.0
            and projected_service_stop_safe_before_svl
            and not stop_position_emergency
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        )
        if low_speed_station_guard and not self._low_speed_guard_active:
            self.log_event("ATP_NUISANCE_GUARD_LOW_SPEED", "safe_projected_stop_before_svl")
            self._low_speed_guard_active = True
        current_ebi_violation = (
            ebi_speed_violation
            and not low_speed_station_guard
            and not low_speed_departure_hold_guard
            and not precise_stop_service_guard
        )
        emergency_condition = emergency or current_ebi_violation
        atp_emergency_reason = ""
        if emergency_condition:
            if self.emergency_stop:
                atp_emergency_reason = "requested_emergency"
            elif stop_position_emergency:
                atp_emergency_reason = "stop_position_or_eoa"
            elif current_ebi_violation:
                atp_emergency_reason = "over_ebi_or_ebd"
            else:
                atp_emergency_reason = "protection_condition"
        if emergency_condition:
            self.emg_latch = True
            accel = -a_emergency
            self.atp_brake = "EMERGENCY"
        else:
            if service_intervention:
                accel = -a_service
                self.atp_brake = "SERVICE"
            elif self.commanded_stop:
                if stop_overshoot:
                    accel = -a_service
                    self.atp_brake = "NONE"
                elif (
                    0.0 < actual_distance_to_stop <= max(STOP_ACCURACY_TOL_M + 0.2, STOP_ACCURACY_TOL_M)
                    and control_speed <= kmh_to_ms(3.0)
                    and self.ato_target_speed <= kmh_to_ms(3.0)
                    and self.jog_state == JOG_STATE_IDLE
                ):
                    # Avoid low-speed hunting only in the sub-metre capture band.
                    # Farther out, ATO must keep creeping instead of stopping short.
                    accel = -min(a_service, COAST_BASE_DECEL)
                    self.atp_brake = "NONE"
                else:
                    error = self.ato_target_speed - control_speed
                    accel = self.compute_ato_pid_accel(error, control_speed, a_service, a_traction)
                    self.atp_brake = "NONE"
            else:
                if self.vital_speed > self.cutoff_threshold:
                    # Hidden traction power cut threshold between W and SBD.
                    self.traction_cutoff = True
                    accel = -coast_decel
                    self.atp_brake = "CUT_POWER"
                else:
                    # ATO follows its own piloting curve while ATP keeps supervising above it.
                    error = self.ato_target_speed - control_speed
                    accel = self.compute_ato_pid_accel(error, control_speed, a_service, a_traction)
                    self.atp_brake = "NONE"

        if self.emg_latch:
            accel = -a_emergency
            self.atp_brake = "EMERGENCY"

        if self.emergency_recovery_hold:
            accel = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else -a_service
            self.atp_brake = "HOLD"

        if stop_ebi_floor_active and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M:
            self.ato_target_speed = 0.0
            self.traction_cutoff = True
            accel = 0.0 if self.speed <= STANDSTILL_SPEED_EPS else -min(a_service, COAST_BASE_DECEL)
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        if hard_departure_hold:
            self.ato_target_speed = 0.0
            if self.speed <= kmh_to_ms(1.0):
                accel = 0.0
                self.speed = 0.0
                self.filtered_speed = 0.0
                self.prev_accel = 0.0
                if not self.standstill_required:
                    self.standstill_anchor_pos = self.pos
                self.standstill_required = True
            else:
                accel = -a_service
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        if (
            self.commanded_stop
            and self.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and not temporary_authority_stop
            and self.speed <= STANDSTILL_SPEED_EPS
        ):
            accel = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        hold_position_lock = False
        if (
            (self.dwell_remaining_s > 0.0 or hard_departure_hold)
            and self.standstill_required
            and self.speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
        ):
            hold_position_lock = True
            accel = 0.0
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.pos = self.standstill_anchor_pos
            if not self.emg_latch:
                self.atp_brake = "HOLD"

        accel = limit_jerk(self.prev_accel, accel, MAX_JERK_MS3, DT)
        self.prev_accel = accel
        if self.speed > STANDSTILL_SPEED_EPS or accel > 0.0:
            accel -= running_resistance_accel_ms2(self.speed)
        accel -= G * self.gradient

        prev_speed = self.speed
        next_speed = max(0.0, prev_speed + accel * DT)
        self.speed = next_speed
        moving_this_step = prev_speed > STANDSTILL_SPEED_EPS or next_speed > STANDSTILL_SPEED_EPS

        if prev_speed <= STANDSTILL_SPEED_EPS and next_speed <= STANDSTILL_SPEED_EPS:
            self.pos = self.pos
        else:
            self.pos = self.pos + 0.5 * (prev_speed + next_speed) * DT

        if hold_position_lock:
            self.pos = self.standstill_anchor_pos
            self.speed = 0.0
            self.prev_accel = 0.0
            moving_this_step = False

        # Force emergency if the train rolls back without authorization.
        if not self.commanded_stop and self.pos < self.prev_pos - ROLLBACK_PROTECT_M:
            self.rollback_protection = True
            self.emg_latch = True
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "ROLLBACK"
            self.atp_brake = "EMERGENCY"
            self.atp_action = "EBI"

        # During the single recovery jog, clamp the train exactly onto the stop target
        # as soon as it reaches or crosses it, so no second jog is needed.
        if (
            moving_this_step
            and self.jog_state == JOG_STATE_ACTIVE
            and self.prev_pos < self.stop_target_pos <= self.pos
            and prev_speed <= kmh_to_ms(JOG_SPEED_KMH + 0.5)
            and not self.standstill_required
        ):
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
            self._set_jog_state(JOG_STATE_COMPLETED, "clamped_at_target")
            self.jog_active = False
        # Snap to the stop target when the train is already in the final low-speed docking window.
        elif (
            moving_this_step
            and self.jog_state not in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and abs(self.stop_target_pos - self.pos) <= STOP_ACCURACY_TOL_M
            and control_speed <= kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH)
            and not self.standstill_required
        ):
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            if self.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE):
                self.standstill_required = True
                self.standstill_anchor_pos = self.pos
                self._set_jog_state(JOG_STATE_COMPLETED, "snapped_within_tolerance")
                self.jog_active = False
        elif (
            self.commanded_stop
            and self.jog_state == JOG_STATE_IDLE
            and not self.standstill_required
            and not self.emg_latch
            and not self.trip_mode
            and self.speed <= STANDSTILL_SPEED_EPS
            and 0.0 <= self.stop_target_pos - self.pos <= MANUAL_JOG_WINDOW_M
        ):
            # The final anti-hunting coast can settle just short of the marker.
            # Close that sub-meter gap once stopped so dwell/door logic sees an exact stop.
            self.pos = self.stop_target_pos
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
        elif self.jog_state == JOG_STATE_ACTIVE and self.pos > self.stop_target_pos + STOP_ACCURACY_TOL_M:
            self.speed = 0.0
            self.ato_target_speed = 0.0
            self.prev_accel = 0.0
            self._set_jog_state(JOG_STATE_FAILED_LOCKED, "overshot_target")
        # Standstill supervision should only hold the train once it has effectively reached the stop area
        # or after an emergency latch. A scheduled stop far ahead must not anchor standstill at the origin.
        stop_pending = self.commanded_stop and actual_distance_to_stop > STOP_ACCURACY_TOL_M
        stop_hold_zone = self.commanded_stop and actual_distance_to_stop <= max(
            STOP_ACCURACY_TOL_M,
            JOG_MAX_DIST_M,
            RELEASE_HANDOVER_START_M,
        )
        must_hold_standstill = (
            (stop_hold_zone or self.emg_latch)
            and self.speed <= STANDSTILL_SPEED_EPS
            and not stop_pending
        )
        if must_hold_standstill and not self.standstill_required:
            self.standstill_required = True
            self.standstill_anchor_pos = self.pos
        elif (
            stop_pending
            and actual_distance_to_stop > self.docking_jog_limit_m() + JOG_WINDOW_EPS_M
            and self.dwell_remaining_s <= 0.0
            and not self.emg_latch
            and not self.trip_mode
            and not self.emergency_recovery_hold
        ):
            self.standstill_required = False
        elif not (self.commanded_stop or self.emg_latch):
            self.standstill_required = False

        waiting_precise_jog = (
            self.commanded_stop
            and self.speed <= STANDSTILL_SPEED_EPS
            and STOP_ACCURACY_TOL_M < actual_distance_to_stop <= JOG_MAX_DIST_M
            and not self.emergency_stop
            and not self.trip_mode
        )
        if waiting_precise_jog and self.standstill_required:
            # A train stopped short of the platform marker is allowed to wait for
            # precise jog. Do not escalate this waiting state into EBI.
            self.standstill_anchor_pos = self.pos

        if (
            self.standstill_required
            and not waiting_precise_jog
            and abs(self.pos - self.standstill_anchor_pos) > STANDSTILL_DRIFT_M
        ):
            self.emg_latch = True
            self.atp_state = "ATP_EMERGENCY"
            self.atp_alert = "STANDSTILL DRIFT"
            self.atp_brake = "EMERGENCY"
            self.atp_action = "EBI"

        self.zero_speed_detected = self.speed <= STANDSTILL_SPEED_EPS
        self.standstill_monitoring = self.standstill_required or self.zero_speed_detected
        self.rollback_monitoring = self.zero_speed_detected or self.commanded_stop or self.emg_latch
        brake_applied_for_hold = self.standstill_required or self.service_brake_latch or self.emg_latch or self.commanded_stop
        station_target_pos = self.scheduled_stop_target_pos()
        station_stop_ready = (
            station_target_pos is None
            or (
                self.station_lane is not None
                and self.stop_target_pos >= station_target_pos - JOG_WINDOW_EPS_M
                and abs(self.pos - station_target_pos) <= STOP_ACCURACY_TOL_M
            )
        )
        self.door_authorized = (
            self.zero_speed_detected
            and brake_applied_for_hold
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
            and station_stop_ready
            and abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M
        )
        self.door_open_allowed = self.door_authorized
        self.ato_hold_active = self.zero_speed_detected and (
            stop_hold_zone or self.standstill_required or self.ato_target_speed <= STANDSTILL_SPEED_EPS
        )
        if self.door_authorized:
            self.ato_door_mode = "ENABLE"
        elif self.ato_hold_active:
            self.ato_door_mode = "READY"
        else:
            self.ato_door_mode = "LOCKED"

        if self.jog_active:
            self.precise_stop_state = "JOG"
        elif self.jog_state == JOG_STATE_FAILED_LOCKED:
            self.precise_stop_state = "JOG_FAILED"
        elif abs(self.pos - self.stop_target_pos) <= STOP_ACCURACY_TOL_M:
            self.precise_stop_state = "ALIGNED"
        elif self.commanded_stop and self.zero_speed_detected:
            self.precise_stop_state = "WAIT_JOG"
        else:
            self.precise_stop_state = "TRACKING"

        if self.trip_mode:
            self.atp_state = "ATP_TRIP"
        elif emergency or self.emergency_stop or self.emg_latch or current_ebi_violation:
            self.atp_state = "ATP_EMERGENCY"
        elif self.emergency_recovery_hold:
            self.atp_state = "ATP_RECOVERY_HOLD"
        elif self.departure_hold and self.zero_speed_detected:
            self.atp_state = "ATP_STANDSTILL"
        elif stop_ebi_floor_active:
            self.atp_state = "ATP_STANDSTILL"
        elif self.zero_speed_detected and self.standstill_required:
            self.atp_state = "ATP_STANDSTILL"
        elif self.zero_speed_detected:
            self.atp_state = "ATP_STANDBY"
        elif service_intervention and not stop_ebi_floor_active:
            self.atp_state = "ATP_SERVICE"
        elif self.vital_speed > self.cutoff_threshold + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_state = "ATP_CUTOFF"
        elif self.vital_speed > self.curves["W"] + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_state = "ATP_WARNING"
        else:
            self.atp_state = "ATP_OK"

        if self.trip_mode:
            self.atp_alert = self.trip_reason or "TRAIN TRIP"
        elif self.emergency_recovery_hold:
            self.atp_alert = "SAFE CONFIRMED"
        elif self.departure_hold:
            self.atp_alert = "DEPARTURE HOLD"
        elif self.standstill_required and self.emg_latch and abs(self.pos - self.standstill_anchor_pos) > STANDSTILL_DRIFT_M:
            self.atp_alert = "STANDSTILL DRIFT"
        elif stop_ebi_floor_active:
            self.atp_alert = "STANDSTILL MON"
        elif self.door_authorized:
            self.atp_alert = "STANDSTILL MON"
        elif self.zero_speed_detected and self.standstill_required:
            self.atp_alert = "STANDSTILL MON"
        elif self.zero_speed_detected:
            self.atp_alert = "ZERO SPEED"
        elif current_ebi_violation and self.vital_speed > self.hidden_curves["EBI"] + 0.05:
            self.atp_alert = "OVER EBI"
        elif current_ebi_violation and self.vital_speed > self.curves["EBD"] + 0.05:
            self.atp_alert = "OVER EBD"
        elif service_intervention and not stop_ebi_floor_active:
            self.atp_alert = "OVER SBI"
        elif self.vital_speed > self.cutoff_threshold + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_alert = "OFF ENERGY"
        elif self.vital_speed > self.curves["W"] + 0.05 and not low_speed_service_guard and not stop_ebi_floor_active:
            self.atp_alert = "OVER W"
        else:
            self.atp_alert = "OK"

        if self.atp_state in ("ATP_EMERGENCY", "ATP_TRIP"):
            self.atp_action = "EBI"
        elif self.atp_state == "ATP_SERVICE":
            self.atp_action = "SBI"
        elif self.atp_state == "ATP_CUTOFF":
            self.atp_action = "OFF"
        elif self.atp_state == "ATP_WARNING":
            self.atp_action = "WARN"
        else:
            self.atp_action = ""

        service_active_now = self.atp_action == "SBI"
        emergency_active_now = self.atp_action == "EBI"
        if service_active_now and not self._last_atp_service_active:
            self.log_event("ATP_SERVICE_INTERVENTION", "over_sbi_or_service_latch")
        if emergency_active_now and not self._last_atp_emergency_active:
            self.log_event("ATP_EMERGENCY_INTERVENTION", atp_emergency_reason or self.atp_alert)
        if self.door_authorized and not self._last_door_authorized and self.jog_state == JOG_STATE_COMPLETED:
            self.log_event("DOOR_AUTHORIZED_AFTER_JOG")
        self._last_atp_service_active = service_active_now
        self._last_atp_emergency_active = emergency_active_now
        self._last_door_authorized = self.door_authorized

        if self.emergency_recovery_hold:
            self.ato_state = "ATO_HOLD"
        elif self.drive_mode == "CMD25" and not self.zero_speed_detected:
            self.ato_state = "RM/CMD25"
        elif self.drive_mode == "LMD" and not self.zero_speed_detected:
            self.ato_state = "LMD"
        elif self.jog_active:
            self.ato_state = "ATO_JOG"
        elif self.ato_hold_active:
            self.ato_state = "ATO_HOLD"
        elif self.zero_speed_detected:
            self.ato_state = "ATO_STANDBY"
        elif self.release_active and control_speed <= kmh_to_ms(CREEP_MAX_SPEED_KMH):
            self.ato_state = "ATO_CREEP"
        elif self.commanded_stop:
            self.ato_state = "ATO_STOP"
        elif self.speed < self.ato_target_speed - 0.1:
            self.ato_state = "ATO_TRACTION"
        elif self.speed > self.ato_target_speed + 0.1:
            self.ato_state = "ATO_BRAKE"
        else:
            self.ato_state = "ATO_COAST"
        if (
            self.commanded_stop
            and abs(actual_distance_to_stop) <= STOP_ACCURACY_TOL_M
            and self.vital_speed <= kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH)
            and not self.emergency_stop
            and not self.emg_latch
            and not self.trip_mode
        ):
            self.hidden_curves["EBI"] = max(self.hidden_curves["EBI"], kmh_to_ms(STOP_EBI_SUPERVISION_FLOOR_KMH))
            self.curves["EBD"] = max(self.curves["EBD"], self.hidden_curves["EBI"])
        if self.ato_fault_active:
            self.apply_ato_unavailable()
        elif self.ato_recovery_state == "READY":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_READY"
            self.ato_target_speed = 0.0
        elif self.ato_recovery_state == "CONFIRMED":
            self.drive_mode = "LMD"
            self.ato_state = "ATO_CONFIRMED"
            self.ato_target_speed = 0.0
        if self.atp_fault_active:
            self.atp_state = "ATP_TRIP"
            self.atp_alert = "ATP FAULT"
            self.atp_action = "EBI"
            self.atp_brake = "EMERGENCY"
            self.emg_latch = True
        distance_delta_m = max(0.0, self.pos - self.prev_pos)
        self.analytics_distance_m += distance_delta_m
        if self.atp_brake in {"SERVICE", "EMERGENCY", "HOLD"} or self.ato_brake_mode not in {"none", "coast"}:
            self.runtime_brake_force_n = max(0.0, abs(self.prev_accel) * self.mass)
            self.runtime_traction_force_n = 0.0
        elif self.speed > 0.0 and not self.traction_cutoff:
            self.runtime_traction_force_n = max(0.0, self.prev_accel * self.mass)
            self.runtime_brake_force_n = 0.0
        else:
            self.runtime_traction_force_n = 0.0
            self.runtime_brake_force_n = 0.0
        self.analytics_traction_work_j += self.runtime_traction_force_n * distance_delta_m
        self.analytics_brake_work_j += self.runtime_brake_force_n * distance_delta_m
        self.prev_pos = self.pos
        self.prev_commanded_stop = self.commanded_stop



__all__ = ["Train", "train_color"]
