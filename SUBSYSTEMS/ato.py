from __future__ import annotations

from dataclasses import dataclass

from CONFIG.config import BRAKE_BUILDUP_S, DT
from SUBSYSTEMS.control_common import (
    ATO_TARGET_DROP_RATE_KMH_S,
    CREEP_MAX_SPEED_KMH,
    DOCKING_SPEED_KMH,
    DOCKING_ZONE_M,
    DOWNHILL_P_BUFFER_KMH_PER_GRAD,
    FINAL_CREEP_MIN_SPEED_KMH,
    FINAL_CREEP_ZONE_M,
    JOG_MAX_DIST_M,
    JOG_SPEED_KMH,
    JOG_STATE_ACTIVE,
    JOG_STATE_COMPLETED,
    JOG_STATE_FAILED_LOCKED,
    JOG_STATE_REQUESTED,
    RELEASE_HANDOVER_START_M,
    RELEASE_SCAN_FAST_KMH,
    RELEASE_SPEED_KMH,
    STANDSTILL_SPEED_EPS,
    STOP_ACCURACY_TOL_M,
    ato_ebi_guard_ms,
    ato_tracking_margin_ms,
    lerp,
    release_speed_profile,
)
from SUBSYSTEMS.physics import kmh_to_ms, traction_acceleration_ms2
from SUBSYSTEMS.signalling import max_speed_with_buildup


@dataclass
class ATOPilotingResult:
    ato_brake_prepare: bool
    ato_curve_speed: float
    ato_piloting_speed: float
    ato_target_speed: float
    jog_active: bool
    jog_used: bool
from .atp import ATPEnvelopeResult


class ATOPilotingEngine:
    """Non-vital operating target generator that runs beneath the ATP envelope."""

    def compute(self, train: "Train", atp: ATPEnvelopeResult) -> ATOPilotingResult:
        ato_guard_limit = max(0.0, atp.hidden_curves["EBI"] - ato_ebi_guard_ms(train.vital_speed))
        ato_brake_prepare = atp.target_active and train.vital_speed >= max(0.0, atp.curves["P"] - kmh_to_ms(0.2))
        ato_curve_speed = min(atp.curves["P"], ato_guard_limit)
        ato_stop_limit = max_speed_with_buildup(
            atp.actual_distance_to_stop,
            atp.a_service,
            BRAKE_BUILDUP_S,
        )
        ato_tracking_margin = ato_tracking_margin_ms(atp.control_speed)
        ato_piloting_speed = max(
            0.0,
            ato_curve_speed
            - ato_tracking_margin
            - kmh_to_ms(max(0.0, -train.gradient) * DOWNHILL_P_BUFFER_KMH_PER_GRAD),
        )
        release_target = min(
            ato_piloting_speed,
            release_speed_profile(atp.actual_distance_to_stop, kmh_to_ms(RELEASE_SPEED_KMH)),
        )
        design_speed_limit = kmh_to_ms(float(getattr(train, "max_ato_speed_kmh", 70.0)))
        desired_target = min(ato_piloting_speed, design_speed_limit)
        if atp.target_active or train.commanded_stop or atp.release_active:
            desired_target = min(desired_target, ato_stop_limit)
        if atp.target_active and not train.commanded_stop and not atp.release_active:
            if train.vital_speed > atp.curves["W"]:
                desired_target = min(desired_target, max(0.0, atp.curves["P"] - kmh_to_ms(1.0)))
            elif train.vital_speed > atp.curves["P"]:
                desired_target = min(desired_target, max(0.0, atp.curves["P"] - kmh_to_ms(2.0)))
        if atp.release_active:
            desired_target = min(desired_target, release_target)
        if atp.actual_distance_to_stop <= DOCKING_ZONE_M:
            if atp.actual_distance_to_stop <= STOP_ACCURACY_TOL_M:
                docking_limit = 0.0
            elif atp.actual_distance_to_stop <= FINAL_CREEP_ZONE_M:
                span_m = max(0.1, FINAL_CREEP_ZONE_M - STOP_ACCURACY_TOL_M)
                ratio = max(0.0, min(1.0, (atp.actual_distance_to_stop - STOP_ACCURACY_TOL_M) / span_m))
                docking_limit = lerp(0.0, kmh_to_ms(FINAL_CREEP_MIN_SPEED_KMH), ratio ** 0.7)
            else:
                span_m = max(0.1, DOCKING_ZONE_M - FINAL_CREEP_ZONE_M)
                ratio = max(0.0, min(1.0, (atp.actual_distance_to_stop - FINAL_CREEP_ZONE_M) / span_m))
                docking_limit = lerp(kmh_to_ms(FINAL_CREEP_MIN_SPEED_KMH), kmh_to_ms(DOCKING_SPEED_KMH), ratio ** 0.9)
            desired_target = min(desired_target, docking_limit)
        if atp.release_active:
            desired_target = min(desired_target, kmh_to_ms(CREEP_MAX_SPEED_KMH))
        if train.commanded_stop and STOP_ACCURACY_TOL_M < atp.actual_distance_to_stop <= RELEASE_HANDOVER_START_M:
            desired_target = min(
                desired_target,
                release_speed_profile(atp.actual_distance_to_stop, kmh_to_ms(RELEASE_SCAN_FAST_KMH)),
            )
        if (
            train.commanded_stop
            and train.jog_state in (JOG_STATE_COMPLETED, JOG_STATE_FAILED_LOCKED)
            and atp.actual_distance_to_stop > STOP_ACCURACY_TOL_M
            and train.speed <= STANDSTILL_SPEED_EPS
        ):
            # After the single permitted jog has fully finished and the train is
            # back at standstill, do not let the final-approach minimum-speed
            # floor start another unintended creep.
            desired_target = 0.0
        ato_target_speed = train.ato_target_speed
        if desired_target > ato_target_speed:
            max_rise = traction_acceleration_ms2(train.speed) * DT
            ato_target_speed = min(desired_target, ato_target_speed + max_rise)
        else:
            max_drop = kmh_to_ms(ATO_TARGET_DROP_RATE_KMH_S) * DT if train.commanded_stop else float("inf")
            ato_target_speed = max(desired_target, ato_target_speed - max_drop)

        jog_active = False
        jog_used = train.jog_used
        if (
            train.commanded_stop
            and train.jog_state in (JOG_STATE_REQUESTED, JOG_STATE_ACTIVE)
            and STOP_ACCURACY_TOL_M < atp.actual_distance_to_stop <= JOG_MAX_DIST_M
        ):
            jog_active = True
            jog_used = True
            ato_target_speed = max(ato_target_speed, kmh_to_ms(JOG_SPEED_KMH))
        elif atp.actual_distance_to_stop <= STOP_ACCURACY_TOL_M or atp.actual_distance_to_stop < 0.0:
            jog_active = False
        if not train.commanded_stop:
            jog_used = False

        return ATOPilotingResult(
            ato_brake_prepare=ato_brake_prepare,
            ato_curve_speed=ato_curve_speed,
            ato_piloting_speed=ato_piloting_speed,
            ato_target_speed=ato_target_speed,
            jog_active=jog_active,
            jog_used=jog_used,
        )


__all__ = ["ATOPilotingEngine", "ATOPilotingResult"]
