from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from CONFIG import config as _config
from SUBSYSTEMS.signalling import (
    VitalBrakeModel,
    conservative_brake_decel_ms2,
    max_entry_speed_with_buildup,
    max_speed_with_buildup,
    stopping_distance_with_buildup,
    vital_delay_margin_m,
)
from SUBSYSTEMS.physics import (
    equivalent_mass_adjusted_accel,
    kmh_to_ms,
    ms_to_kmh,
    traction_acceleration_ms2,
)

for _name in dir(_config):
    if _name.isupper():
        globals()[_name] = getattr(_config, _name)
del _name


def max_speed_for_target(v_target_ms: float, distance_m: float, decel: float) -> float:
    if distance_m <= 0 or decel <= 0:
        return v_target_ms
    return (v_target_ms * v_target_ms + 2.0 * decel * distance_m) ** 0.5


def quantize_speed_ms(speed_ms: float, resolution_kmh: float) -> float:
    if speed_ms <= 0.0 or resolution_kmh <= 0.0:
        return 0.0
    resolution_ms = kmh_to_ms(resolution_kmh)
    steps = round(speed_ms / resolution_ms)
    return max(0.0, steps * resolution_ms)


def lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, ratio))


def low_pass_step(previous: float, measurement: float, tau_s: float, dt_s: float) -> float:
    if tau_s <= 0.0 or dt_s <= 0.0:
        return measurement
    alpha = max(0.0, min(1.0, dt_s / (tau_s + dt_s)))
    return previous + alpha * (measurement - previous)


def release_transition_ratio(distance_m: float) -> float:
    if distance_m <= FINAL_STOP_BRAKE_ZONE_M:
        return 1.0
    if distance_m >= RELEASE_HANDOVER_START_M:
        return 0.0
    span_m = max(0.1, RELEASE_HANDOVER_START_M - FINAL_STOP_BRAKE_ZONE_M)
    ratio = 1.0 - ((distance_m - FINAL_STOP_BRAKE_ZONE_M) / span_m)
    # Smoothstep keeps the final release handover gradual instead of dropping
    # the ATP curves abruptly while ATO is still braking toward the marker.
    return ratio * ratio * (3.0 - 2.0 * ratio)


def release_speed_profile(distance_m: float, release_speed_ms: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    fine_scan_speed_ms = kmh_to_ms(RELEASE_SCAN_FINE_KMH)
    fast_scan_speed_ms = min(release_speed_ms, kmh_to_ms(RELEASE_SCAN_FAST_KMH))
    min_creep_speed_ms = kmh_to_ms(FINAL_APPROACH_MIN_SPEED_KMH)
    if distance_m >= RELEASE_HANDOVER_START_M:
        return fast_scan_speed_ms
    if distance_m >= FINAL_STOP_BRAKE_ZONE_M:
        span_m = max(0.1, RELEASE_HANDOVER_START_M - FINAL_STOP_BRAKE_ZONE_M)
        ratio = max(0.0, min(1.0, (distance_m - FINAL_STOP_BRAKE_ZONE_M) / span_m))
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        return lerp(fine_scan_speed_ms, fast_scan_speed_ms, smooth)
    span_m = max(0.1, FINAL_STOP_BRAKE_ZONE_M - STOP_ACCURACY_TOL_M)
    ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
    smooth = ratio * ratio * (3.0 - 2.0 * ratio)
    return lerp(min_creep_speed_ms, fine_scan_speed_ms, smooth)


def release_entry_speed_limit(distance_m: float, release_speed_ms: float, decel: float) -> float:
    if distance_m <= 0.0:
        return 0.0
    if distance_m <= FINAL_STOP_BRAKE_ZONE_M:
        return release_speed_profile(distance_m, release_speed_ms)
    cruise_distance_m = max(0.0, distance_m - FINAL_STOP_BRAKE_ZONE_M)
    return max_entry_speed_with_buildup(release_speed_ms, cruise_distance_m, decel, BRAKE_BUILDUP_S)


def precise_stop_sbi_limit_ms(distance_m: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    if distance_m <= PRECISE_STOP_SERVICE_BAND_M:
        span_m = max(0.1, PRECISE_STOP_SERVICE_BAND_M - STOP_ACCURACY_TOL_M)
        ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
        return kmh_to_ms(PRECISE_STOP_SBI_ENTRY_KMH) * (ratio ** 0.8)
    if distance_m <= JOG_MAX_DIST_M:
        span_m = max(0.1, JOG_MAX_DIST_M - PRECISE_STOP_SERVICE_BAND_M)
        ratio = max(0.0, min(1.0, (distance_m - PRECISE_STOP_SERVICE_BAND_M) / span_m))
        sbi_kmh = PRECISE_STOP_SBI_ENTRY_KMH + (FINAL_APPROACH_SBI_FLOOR_KMH - PRECISE_STOP_SBI_ENTRY_KMH) * ratio
        return kmh_to_ms(sbi_kmh)
    return kmh_to_ms(FINAL_APPROACH_SBI_FLOOR_KMH)


def precise_stop_gap_ms(distance_m: float, full_gap_kmh: float) -> float:
    if distance_m <= STOP_ACCURACY_TOL_M:
        return 0.0
    span_m = max(0.1, PRECISE_STOP_SERVICE_BAND_M - STOP_ACCURACY_TOL_M)
    ratio = max(0.0, min(1.0, (distance_m - STOP_ACCURACY_TOL_M) / span_m))
    return kmh_to_ms(full_gap_kmh) * (ratio ** 0.8)


def precise_stop_profile_active(train: "Train", distance_m: float) -> bool:
    if not train.commanded_stop or train.trip_mode:
        return False
    if distance_m < 0.0 or distance_m > JOG_MAX_DIST_M:
        return False
    return (
        distance_m <= PRECISE_STOP_SERVICE_BAND_M
        or train.zero_speed_detected
        or train.speed <= STANDSTILL_SPEED_EPS
    )


def emergency_speed_curve(
    target_speed_ms: float,
    distance_m: float,
    decel: float,
    reaction_margin_m: float,
) -> float:
    """Compute the emergency intervention curve (EBI/EBD) from an EBD baseline."""
    if decel <= 0 or distance_m <= 0:
        return 0.0
    d = distance_m - (POS_UNCERT_M + reaction_margin_m)
    return max(0.0, max_entry_speed_with_buildup(target_speed_ms, max(0.0, d), decel, BRAKE_BUILDUP_S))


def ato_tracking_margin_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = ATO_TRACKING_MARGIN_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = ATO_TRACKING_MARGIN_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = ATO_TRACKING_MARGIN_LOW_KMH + ratio * (
            ATO_TRACKING_MARGIN_HIGH_KMH - ATO_TRACKING_MARGIN_LOW_KMH
        )
    low_speed_relief = ATO_TRACKING_MARGIN_LOW_SPEED_RELIEF_KMH * low_speed_flexibility_scale(speed_ms)
    margin_kmh = max(ATO_TRACKING_MARGIN_MIN_LOW_KMH, margin_kmh - low_speed_relief)
    return kmh_to_ms(margin_kmh)


def i_curve_margin_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = I_CURVE_MARGIN_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = I_CURVE_MARGIN_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = I_CURVE_MARGIN_LOW_KMH + ratio * (
            I_CURVE_MARGIN_HIGH_KMH - I_CURVE_MARGIN_LOW_KMH
        )
    return kmh_to_ms(margin_kmh)


def ato_ebi_guard_ms(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_TRACKING_MARGIN_BLEND_FROM_KMH:
        margin_kmh = ATO_EBI_GUARD_LOW_KMH
    elif speed_kmh >= ATO_TRACKING_MARGIN_BLEND_TO_KMH:
        margin_kmh = ATO_EBI_GUARD_HIGH_KMH
    else:
        ratio = (
            (speed_kmh - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
            / (ATO_TRACKING_MARGIN_BLEND_TO_KMH - ATO_TRACKING_MARGIN_BLEND_FROM_KMH)
        )
        margin_kmh = ATO_EBI_GUARD_LOW_KMH + ratio * (
            ATO_EBI_GUARD_HIGH_KMH - ATO_EBI_GUARD_LOW_KMH
        )
    return kmh_to_ms(margin_kmh)


def low_speed_flexibility_scale(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= LOW_SPEED_FLEX_FULL_KMH:
        return 1.0
    if speed_kmh >= LOW_SPEED_FLEX_NONE_KMH:
        return 0.0
    return 1.0 - (
        (speed_kmh - LOW_SPEED_FLEX_FULL_KMH)
        / (LOW_SPEED_FLEX_NONE_KMH - LOW_SPEED_FLEX_FULL_KMH)
    )

def ato_brake_gain(speed_ms: float) -> float:
    speed_ratio = min(1.0, max(0.0, (ms_to_kmh(speed_ms) - 40.0) / 60.0))
    low_speed_relief = 0.2 * low_speed_flexibility_scale(speed_ms)
    return max(0.85, 1.05 + 0.45 * speed_ratio - low_speed_relief)


def ato_pid_gains(speed_ms: float) -> Tuple[float, float, float]:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= ATO_PID_BLEND_FROM_KMH:
        return ATO_PID_KP_LOW, ATO_PID_KI_LOW, ATO_PID_KD_LOW
    if speed_kmh >= ATO_PID_BLEND_TO_KMH:
        return ATO_PID_KP_HIGH, ATO_PID_KI_HIGH, ATO_PID_KD_HIGH
    ratio = (speed_kmh - ATO_PID_BLEND_FROM_KMH) / max(0.1, ATO_PID_BLEND_TO_KMH - ATO_PID_BLEND_FROM_KMH)
    return (
        lerp(ATO_PID_KP_LOW, ATO_PID_KP_HIGH, ratio),
        lerp(ATO_PID_KI_LOW, ATO_PID_KI_HIGH, ratio),
        lerp(ATO_PID_KD_LOW, ATO_PID_KD_HIGH, ratio),
    )


def high_speed_curve_scale(speed_ms: float) -> float:
    speed_kmh = ms_to_kmh(speed_ms)
    if speed_kmh <= HIGH_SPEED_CURVE_BLEND_FROM_KMH:
        return 0.0
    if speed_kmh >= HIGH_SPEED_CURVE_BLEND_TO_KMH:
        return 1.0
    return (
        (speed_kmh - HIGH_SPEED_CURVE_BLEND_FROM_KMH)
        / (HIGH_SPEED_CURVE_BLEND_TO_KMH - HIGH_SPEED_CURVE_BLEND_FROM_KMH)
    )


def required_brake_rate_for_target(current_speed_ms: float, target_speed_ms: float, distance_m: float) -> float:
    """Minimum constant deceleration needed to reach target speed within the remaining distance."""
    if current_speed_ms <= target_speed_ms:
        return 0.0
    if distance_m <= 0.0:
        return float("inf")
    return max(0.0, (current_speed_ms * current_speed_ms - target_speed_ms * target_speed_ms) / (2.0 * distance_m))


def target_curve_reserve_m(current_speed_ms: float, target_speed_ms: float) -> float:
    speed_gap_ratio = 0.0
    if current_speed_ms > 0.0:
        speed_gap_ratio = max(0.0, min(1.0, (current_speed_ms - target_speed_ms) / current_speed_ms))
    high_speed_scale = high_speed_curve_scale(current_speed_ms)
    blend = max(high_speed_scale, speed_gap_ratio)
    return TARGET_CURVE_RESERVE_LOW_M + (TARGET_CURVE_RESERVE_HIGH_M - TARGET_CURVE_RESERVE_LOW_M) * blend


def indication_speed_delta_ms(service_decel_ms2: float, delay_s: float) -> float:
    if service_decel_ms2 <= 0.0 or delay_s <= 0.0:
        return kmh_to_ms(CURVE_EPS_KMH)
    return max(kmh_to_ms(CURVE_EPS_KMH), service_decel_ms2 * delay_s)

__all__ = [name for name in globals() if not name.startswith("_")]
