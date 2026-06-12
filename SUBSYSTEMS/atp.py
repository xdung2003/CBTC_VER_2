from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict

from CONFIG.config import BRAKE_BUILDUP_S, BRAKE_FORCE_N, EMERGENCY_FORCE_N
from SUBSYSTEMS.control_common import (
    ATP_EBI_REACTION_S,
    ATP_INDICATION_DELAY_S,
    ATP_P_REACTION_S,
    ATP_SBI_REACTION_S,
    ATP_SERVICE_BRAKE_FACTOR,
    ATP_W_REACTION_S,
    ATO_CONTROL_RES_KMH,
    ATO_TARGET_PREP_MAX_M,
    CREEP_RELEASE_CAP_KMH,
    CURVE_EPS_KMH,
    EBD_SPEED_TOL_KMH,
    EBI_SPEED_MARGIN_KMH,
    HIGH_SPEED_SPEED_TOL_GAIN_KMH,
    HIGH_SPEED_TIME_MARGIN_GAIN,
    JOG_MAX_DIST_M,
    PRECISE_STOP_EBI_GAP_KMH,
    PRECISE_STOP_SERVICE_BAND_M,
    RELEASE_ENTRY_MARGIN_KMH,
    RELEASE_SPEED_KMH,
    RELEASE_ZONE_M,
    SBI_SPEED_TOL_KMH,
    SPEED_ESTIMATION_RES_KMH,
    STANDSTILL_SPEED_EPS,
    STOP_ACCURACY_TOL_M,
    STOP_TARGET_BUFFER_M,
    STOP_TARGET_MIN_ACTIVATION_M,
    VITAL_SPEED_MARGIN_KMH,
    W_SPEED_TOL_KMH,
    high_speed_curve_scale,
    indication_speed_delta_ms,
    lerp,
    precise_stop_gap_ms,
    precise_stop_profile_active,
    precise_stop_sbi_limit_ms,
    quantize_speed_ms,
    release_entry_speed_limit,
    release_speed_profile,
    release_transition_ratio,
)
from SUBSYSTEMS.physics import equivalent_mass_adjusted_accel, kmh_to_ms, ms_to_kmh, traction_acceleration_ms2
from SUBSYSTEMS.signalling import (
    ATP_BRAKE_BUILDUP_S,
    ATP_EMERGENCY_BRAKE_FACTOR,
    VitalBrakeModel,
    conservative_brake_decel_ms2,
    stopping_distance_with_buildup,
    vital_delay_margin_m,
)


@dataclass
class ATPEnvelopeResult:
    control_speed: float
    actual_distance_to_stop: float
    distance_to_svl: float
    svl_m: float
    target_active: bool
    stop_target_active: bool
    release_active: bool
    release_blend: float
    p_t: float
    p_r: float
    a_service: float
    a_emergency: float
    a_traction: float
    curves: Dict[str, float]
    hidden_curves: Dict[str, float]
    curve_mode: str
    cutoff_threshold: float
    margin_dyn_m: float
    atp_service_brake_decel: float
    atp_emergency_brake_decel: float
    release_speed_kmh: float = 0.0


class ATPEnvelopeEngine:
    """Vital supervision engine. Computes ATP curves independently from ATO piloting."""

    def compute(self, train: "Train") -> ATPEnvelopeResult:
        psr_ms = kmh_to_ms(train.psr_kmh)
        a_service = equivalent_mass_adjusted_accel(BRAKE_FORCE_N / train.mass)
        a_emergency = equivalent_mass_adjusted_accel(EMERGENCY_FORCE_N / train.mass)
        a_traction = traction_acceleration_ms2(train.speed)
        atp_service_brake_decel = conservative_brake_decel_ms2(
            BRAKE_FORCE_N,
            train.mass,
            ATP_SERVICE_BRAKE_FACTOR,
        )
        atp_emergency_brake_decel = conservative_brake_decel_ms2(
            EMERGENCY_FORCE_N,
            train.mass,
            ATP_EMERGENCY_BRAKE_FACTOR,
        )
        estimated_speed = quantize_speed_ms(train.filtered_speed, SPEED_ESTIMATION_RES_KMH)
        control_speed = quantize_speed_ms(train.filtered_speed, ATO_CONTROL_RES_KMH)
        if train.filtered_speed <= STANDSTILL_SPEED_EPS:
            # Do not keep a residual vital-speed margin at standstill; it prevents
            # the final jog from re-applying traction and can deadlock a station stop.
            estimated_speed = 0.0
            control_speed = 0.0
            vital_speed = 0.0
        else:
            vital_speed = estimated_speed + kmh_to_ms(VITAL_SPEED_MARGIN_KMH)
        position_uncertainty_m = train.effective_position_uncertainty_m()

        brake_model = VitalBrakeModel(
            train.track_profile,
            train.safe_front_end_pos,
            position_uncertainty_m,
            vital_speed,
            ATP_BRAKE_BUILDUP_S,
        )

        svl_pos = brake_model.supervised_location_m(train.eoa)
        distance_to_svl = svl_pos - train.reported_pos
        actual_distance_to_stop = train.stop_target_pos - train.pos
        high_speed_scale = high_speed_curve_scale(vital_speed)
        time_margin_boost = 1.0 + HIGH_SPEED_TIME_MARGIN_GAIN * high_speed_scale
        extra_tol_kmh = HIGH_SPEED_SPEED_TOL_GAIN_KMH * high_speed_scale
        i_margin_ms = indication_speed_delta_ms(
            atp_service_brake_decel,
            ATP_INDICATION_DELAY_S * time_margin_boost,
        )

        margin_p = vital_delay_margin_m(vital_speed, ATP_P_REACTION_S * time_margin_boost)
        margin_w = vital_delay_margin_m(vital_speed, ATP_W_REACTION_S * time_margin_boost)
        margin_sbi = vital_delay_margin_m(vital_speed, ATP_SBI_REACTION_S * time_margin_boost)
        margin_ebi = vital_delay_margin_m(vital_speed, ATP_EBI_REACTION_S * time_margin_boost)
        margin_dyn_m = position_uncertainty_m + margin_w

        p_c = psr_ms
        w_c = psr_ms + kmh_to_ms(W_SPEED_TOL_KMH + 0.4 * extra_tol_kmh)
        off_c = psr_ms + kmh_to_ms(1.5 + 0.7 * extra_tol_kmh)
        sbi_c = psr_ms + kmh_to_ms(SBI_SPEED_TOL_KMH + extra_tol_kmh)
        sbd_c = psr_ms + kmh_to_ms(SBI_SPEED_TOL_KMH + 0.5 + 1.2 * extra_tol_kmh)
        ebd_c = psr_ms + kmh_to_ms(EBD_SPEED_TOL_KMH + 1.4 * extra_tol_kmh)
        ebi_c = max(0.0, ebd_c - kmh_to_ms(EBI_SPEED_MARGIN_KMH + 0.35 * extra_tol_kmh))

        speed_target_active = train.limit_ahead_dist != float("inf")
        if speed_target_active:
            limit_ahead_speed_ms = kmh_to_ms(train.limit_ahead_speed_kmh)
            p_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_P_REACTION_S * time_margin_boost,
            )
            w_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_W_REACTION_S * time_margin_boost,
            )
            off_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                0.5 * (ATP_W_REACTION_S + ATP_SBI_REACTION_S) * time_margin_boost,
            )
            sbi_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            sbd_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_service_brake_decel,
                0.0,
            )
            ebd_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_emergency_brake_decel,
                0.0,
            )
            ebi_speed = brake_model.speed_for_target(
                limit_ahead_speed_ms,
                train.limit_ahead_dist,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            ebi_speed = min(ebi_speed, max(0.0, ebd_speed - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
        else:
            p_speed = p_c
            w_speed = w_c
            off_speed = off_c
            sbi_speed = sbi_c
            sbd_speed = sbd_c
            ebi_speed = ebi_c
            ebd_speed = ebd_c

        stop_activation_distance = max(
            STOP_TARGET_MIN_ACTIVATION_M,
            stopping_distance_with_buildup(train.speed, a_service, BRAKE_BUILDUP_S) + STOP_TARGET_BUFFER_M,
        )
        stop_activation_distance = max(stop_activation_distance, ATO_TARGET_PREP_MAX_M)
        eoa_stop_target_available = math.isfinite(train.distance_to_eoa)
        # EOA is always a vital stop target. Supervising it continuously keeps moving-block ATP
        # curves from stepping down when the train crosses a late activation threshold.
        stop_target_active = (
            train.commanded_stop
            or eoa_stop_target_available
            or train.distance_to_eoa <= stop_activation_distance
        )
        if stop_target_active:
            sbd_stop = brake_model.speed_for_stop(train.distance_to_eoa, atp_service_brake_decel, 0.0)
            ebd_stop = brake_model.speed_for_stop(distance_to_svl, atp_emergency_brake_decel, 0.0)
            ebi_stop = brake_model.speed_for_target(
                0.0,
                distance_to_svl,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            ebi_stop = min(ebi_stop, max(0.0, ebd_stop - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
            w_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_W_REACTION_S * time_margin_boost,
            )
            off_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                0.5 * (ATP_W_REACTION_S + ATP_SBI_REACTION_S) * time_margin_boost,
            )
            sbi_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            p_stop = brake_model.speed_for_stop(
                train.distance_to_eoa,
                atp_service_brake_decel,
                ATP_P_REACTION_S * time_margin_boost,
            )
        else:
            sbd_stop = float("inf")
            ebd_stop = float("inf")
            w_stop = float("inf")
            off_stop = float("inf")
            sbi_stop = float("inf")
            p_stop = float("inf")
            ebi_stop = float("inf")

        p_t = min(p_speed, p_stop)
        w_t = min(w_speed, w_stop)
        off_t = min(off_speed, off_stop)
        sbi_t = min(sbi_speed, sbi_stop)
        sbd_t = min(sbd_speed, sbd_stop)
        ebi_t = min(ebi_speed, ebi_stop)
        ebd_t = min(ebd_speed, ebd_stop)
        if ebd_t <= sbd_t + kmh_to_ms(CURVE_EPS_KMH):
            ebd_t = sbd_t + kmh_to_ms(CURVE_EPS_KMH)
        if ebi_t <= sbi_t + kmh_to_ms(CURVE_EPS_KMH):
            ebi_t = sbi_t + kmh_to_ms(CURVE_EPS_KMH)

        release_speed = kmh_to_ms(RELEASE_SPEED_KMH)
        release_base = release_speed_profile(actual_distance_to_stop, release_speed)
        release_speed_kmh = ms_to_kmh(release_base)  # Actual release speed for display
        release_service_entry = release_entry_speed_limit(
            actual_distance_to_stop,
            release_speed,
            atp_service_brake_decel,
        )
        release_emergency_entry = release_entry_speed_limit(
            actual_distance_to_stop,
            release_speed,
            atp_emergency_brake_decel,
        )
        p_r = release_base
        w_r = p_r + kmh_to_ms(1.0)
        off_r = p_r + kmh_to_ms(1.5)
        sbi_r = p_r + kmh_to_ms(2.5)
        release_ebd = max(release_emergency_entry, release_base)
        release_ebi = max(0.0, release_ebd - kmh_to_ms(EBI_SPEED_MARGIN_KMH))
        release_blend = release_transition_ratio(actual_distance_to_stop)
        p_release = min(p_c, lerp(p_t, p_r, release_blend))
        w_release = min(w_c, lerp(w_t, w_r, release_blend))
        off_release = min(off_c, lerp(off_t, off_r, release_blend))
        sbi_release = min(sbi_c, lerp(sbi_t, sbi_r, release_blend))
        sbd_release = min(sbd_t, lerp(sbd_t, max(sbd_t, release_service_entry), release_blend))
        ebi_release = min(ebi_t, lerp(ebi_t, max(ebi_t, release_ebi), release_blend))
        ebd_release = min(ebd_t, lerp(ebd_t, max(ebd_t, release_ebd), release_blend))
        use_precise_profile = precise_stop_profile_active(train, actual_distance_to_stop)
        if use_precise_profile:
            profile_sbi = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            profile_sbd_gap = precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH)
            profile_ebi_gap = precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            profile_ebd_gap = precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH)
            sbi_release = profile_sbi
            sbd_release = profile_sbi + profile_sbd_gap
            ebi_release = profile_sbi + profile_ebi_gap
            ebd_release = ebi_release + profile_ebd_gap
        elif train.commanded_stop and 0.0 < actual_distance_to_stop <= JOG_MAX_DIST_M:
            sbi_floor = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            ebi_floor = sbi_floor + precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            sbi_release = max(sbi_release, sbi_floor)
            sbd_release = max(
                sbd_release,
                sbi_release + precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH),
            )
            ebi_release = max(ebi_release, ebi_floor)
            ebd_release = max(
                ebd_release,
                ebi_release + precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH),
            )
        i_release = max(0.0, p_release - i_margin_ms)

        target_active = speed_target_active or stop_target_active or p_t < p_c - kmh_to_ms(0.1)
        near_release_zone = STOP_ACCURACY_TOL_M < actual_distance_to_stop <= RELEASE_ZONE_M
        release_entry_ok = vital_speed <= min(
            kmh_to_ms(CREEP_RELEASE_CAP_KMH),
            max(release_service_entry, p_t) + kmh_to_ms(RELEASE_ENTRY_MARGIN_KMH),
        )
        release_active = (
            near_release_zone
            and stop_target_active
            and (release_entry_ok or (train.release_active and train.commanded_stop))
            and not train.emergency_stop
            and not train.emg_latch
        )

        if train.trip_mode:
            trip_distance = max(0.0, train.trip_protect_pos - train.safe_front_end_pos)
            trip_sbd = brake_model.speed_for_stop(trip_distance, atp_service_brake_decel, 0.0)
            trip_sbi = brake_model.speed_for_stop(
                trip_distance,
                atp_service_brake_decel,
                ATP_SBI_REACTION_S * time_margin_boost,
            )
            trip_ebd = brake_model.speed_for_stop(trip_distance, atp_emergency_brake_decel, 0.0)
            trip_ebi = brake_model.speed_for_target(
                0.0,
                trip_distance,
                atp_emergency_brake_decel,
                ATP_EBI_REACTION_S * time_margin_boost,
            )
            trip_ebi = min(trip_ebi, max(0.0, trip_ebd - kmh_to_ms(EBI_SPEED_MARGIN_KMH)))
            curve_mode = "TRIP"
            curves = {"P": 0.0, "W": 0.0, "SBD": trip_sbd, "EBD": trip_ebd}
            hidden_curves = {"I": 0.0, "OFF": 0.0, "SBI": trip_sbi, "EBI": trip_ebi}
        elif release_active:
            curve_mode = "RELEASE"
            curves = {"P": p_release, "W": w_release, "SBD": sbd_release, "EBD": ebd_release}
            hidden_curves = {"I": i_release, "OFF": off_release, "SBI": sbi_release, "EBI": ebi_release}
        elif target_active:
            curve_mode = "TARGET"
            curves = {
                "P": min(p_c, p_t),
                "W": min(w_c, w_t),
                "SBD": min(sbd_c, sbd_t),
                "EBD": min(ebd_c, ebd_t),
            }
            hidden_curves = {
                "I": max(0.0, min(p_c, p_t) - i_margin_ms),
                "OFF": min(off_c, off_t),
                "SBI": min(sbi_c, sbi_t),
                "EBI": min(ebi_c, ebi_t),
            }
        else:
            curve_mode = "CEILING"
            curves = {"P": p_c, "W": w_c, "SBD": sbd_c, "EBD": ebd_c}
            hidden_curves = {"I": max(0.0, p_c - i_margin_ms), "OFF": off_c, "SBI": sbi_c, "EBI": ebi_c}

        if use_precise_profile:
            sbi_limit = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            sbd_gap = precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH)
            ebi_gap = precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            ebd_gap = precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH)
            hidden_curves["SBI"] = sbi_limit
            curves["SBD"] = hidden_curves["SBI"] + sbd_gap
            hidden_curves["EBI"] = hidden_curves["SBI"] + ebi_gap
            curves["EBD"] = hidden_curves["EBI"] + ebd_gap
        elif (
            train.commanded_stop
            and not train.trip_mode
            and PRECISE_STOP_SERVICE_BAND_M < actual_distance_to_stop <= JOG_MAX_DIST_M
        ):
            sbi_floor = precise_stop_sbi_limit_ms(actual_distance_to_stop)
            ebi_floor = sbi_floor + precise_stop_gap_ms(actual_distance_to_stop, PRECISE_STOP_EBI_GAP_KMH)
            hidden_curves["SBI"] = max(hidden_curves["SBI"], sbi_floor)
            curves["SBD"] = max(
                curves["SBD"],
                hidden_curves["SBI"] + precise_stop_gap_ms(actual_distance_to_stop, CURVE_EPS_KMH),
            )
            hidden_curves["EBI"] = max(hidden_curves["EBI"], ebi_floor)
            curves["EBD"] = max(
                curves["EBD"],
                hidden_curves["EBI"] + precise_stop_gap_ms(actual_distance_to_stop, EBI_SPEED_MARGIN_KMH),
            )
        if release_active and train.commanded_stop and actual_distance_to_stop > STOP_ACCURACY_TOL_M:
            release_floor = min(p_c, release_speed_profile(actual_distance_to_stop, release_speed))
            curves["P"] = max(curves["P"], release_floor)
            curves["W"] = max(curves["W"], min(w_c, curves["P"] + kmh_to_ms(1.0)))
            hidden_curves["OFF"] = max(hidden_curves["OFF"], curves["W"] + kmh_to_ms(0.5))
            hidden_curves["SBI"] = max(hidden_curves["SBI"], curves["P"] + kmh_to_ms(2.5))
            curves["SBD"] = max(curves["SBD"], hidden_curves["SBI"] + kmh_to_ms(CURVE_EPS_KMH))
            hidden_curves["EBI"] = max(hidden_curves["EBI"], curves["SBD"] + kmh_to_ms(CURVE_EPS_KMH))
            curves["EBD"] = max(curves["EBD"], hidden_curves["EBI"] + kmh_to_ms(EBI_SPEED_MARGIN_KMH))

        def keep_below(value: float, upper: float) -> float:
            if upper <= 0.0:
                return 0.0
            if value >= upper:
                return max(0.0, upper - kmh_to_ms(CURVE_EPS_KMH))
            return value

        hidden_curves["EBI"] = keep_below(hidden_curves["EBI"], curves["EBD"])
        curves["SBD"] = keep_below(curves["SBD"], hidden_curves["EBI"])
        hidden_curves["SBI"] = keep_below(hidden_curves["SBI"], curves["SBD"])
        hidden_curves["OFF"] = keep_below(hidden_curves["OFF"], hidden_curves["SBI"])
        curves["W"] = keep_below(curves["W"], hidden_curves["OFF"])
        curves["P"] = keep_below(curves["P"], curves["W"])
        i_gap_limit = max(kmh_to_ms(CURVE_EPS_KMH), i_margin_ms)
        hidden_curves["I"] = min(hidden_curves["I"], max(0.0, curves["P"] - i_gap_limit))
        cutoff_threshold = hidden_curves["OFF"]

        train.estimated_speed = estimated_speed
        train.vital_speed = vital_speed

        return ATPEnvelopeResult(
            control_speed=control_speed,
            actual_distance_to_stop=actual_distance_to_stop,
            distance_to_svl=distance_to_svl,
            svl_m=svl_pos,
            target_active=target_active,
            stop_target_active=stop_target_active,
            release_active=release_active,
            release_blend=release_blend,
            p_t=p_t,
            p_r=p_r,
            a_service=a_service,
            a_emergency=a_emergency,
            a_traction=a_traction,
            curves=curves,
            hidden_curves=hidden_curves,
            curve_mode=curve_mode,
            cutoff_threshold=cutoff_threshold,
            margin_dyn_m=margin_dyn_m,
            atp_service_brake_decel=atp_service_brake_decel,
            atp_emergency_brake_decel=atp_emergency_brake_decel,
            release_speed_kmh=release_speed_kmh,
        )


__all__ = ["ATPEnvelopeEngine", "ATPEnvelopeResult"]
