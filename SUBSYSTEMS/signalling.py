from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

from CONFIG.config import (
    AW3_MASS_KG,
    ATP_ADHESION_FACTOR,
    ATP_BRAKE_BUILDUP_S,
    ATP_EMERGENCY_BRAKE_FACTOR,
    ATP_MA_EXTRAPOLATION_S,
    ATP_MIN_DECEL_MS2,
    ATP_POS_REPORT_LATENCY_S,
    BRAKE_BUILDUP_S,
    DT,
    EMERGENCY_FORCE_N,
    G,
    MAX_JERK_MS3,
    OVERLAP_M,
    SAFETY_MARGIN_M,
    STOP_SVL_OFFSET_M,
)
from SUBSYSTEMS.physics import equivalent_mass_adjusted_accel, traction_acceleration_ms2


@dataclass
class SafeMovementPacket:
    """Safe ZC packet delivered to an onboard CC."""

    eoa_m: float
    tsr_kmh: float
    variants: Dict[str, float]
    issued_time_s: float = 0.0


@dataclass
class MovementAuthorityLimit:
    """ZC-calculated movement authority for one train."""

    train_id: str
    mal_m: float
    protected_rear_m: float
    follower_braking_m: float
    follower_projection_m: float
    safety_margin_m: float
    overlap_m: float
    reason: str


def get_track_info(track_profile: List[Tuple[float, float, float, float]], pos_m: float) -> Tuple[float, float]:
    for start_m, end_m, gradient, psr_kmh in track_profile:
        if start_m <= pos_m < end_m:
            return gradient, psr_kmh
    return track_profile[-1][2], track_profile[-1][3]


def braking_curve_profile(decel: float, build_s: float) -> Tuple[float, float]:
    if decel <= 0.0:
        return 0.0, 0.0
    if MAX_JERK_MS3 <= 0.0:
        return max(0.0, build_s), 0.0
    ramp_s = decel / MAX_JERK_MS3
    residual_delay_s = max(0.0, build_s - ramp_s)
    return residual_delay_s, ramp_s


def max_speed_with_buildup(distance_m: float, decel: float, build_s: float) -> float:
    if distance_m <= 0.0 or decel <= 0.0:
        return 0.0
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    linear_term = 2.0 * decel * (residual_delay_s + 0.5 * ramp_s)
    constant_term = 2.0 * decel * distance_m + ((decel * ramp_s) ** 2) / 12.0
    term = linear_term * linear_term + 4.0 * constant_term
    return max(0.0, 0.5 * (-linear_term + term ** 0.5))


def max_entry_speed_with_buildup(v_target_ms: float, distance_m: float, decel: float, build_s: float) -> float:
    if decel <= 0.0 or distance_m <= 0.0:
        return v_target_ms
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    linear_term = 2.0 * decel * (residual_delay_s + 0.5 * ramp_s)
    constant_term = v_target_ms * v_target_ms + 2.0 * decel * distance_m + ((decel * ramp_s) ** 2) / 12.0
    term = linear_term * linear_term + 4.0 * constant_term
    return max(v_target_ms, 0.5 * (-linear_term + term ** 0.5))


def stopping_distance_with_buildup(speed_ms: float, decel: float, build_s: float) -> float:
    if speed_ms <= 0.0 or decel <= 0.0:
        return 0.0
    residual_delay_s, ramp_s = braking_curve_profile(decel, build_s)
    return (
        speed_ms * residual_delay_s
        + (speed_ms * speed_ms) / (2.0 * decel)
        + 0.5 * speed_ms * ramp_s
        - decel * ramp_s * ramp_s / 24.0
    )


def worst_gradient_in_range(
    track_profile: List[Tuple[float, float, float, float]],
    start_pos_m: float,
    distance_m: float,
) -> float:
    if not track_profile:
        return 0.0
    if distance_m <= 0.0:
        return get_track_info(track_profile, start_pos_m)[0]
    end_pos_m = start_pos_m + distance_m
    worst_gradient = get_track_info(track_profile, start_pos_m)[0]
    for start_m, end_m, gradient, _psr in track_profile:
        if end_m < start_pos_m or start_m > end_pos_m:
            continue
        worst_gradient = min(worst_gradient, gradient)
    return worst_gradient


def conservative_brake_decel_ms2(force_n: float, mass_kg: float, brake_factor: float) -> float:
    worst_mass_kg = max(mass_kg, AW3_MASS_KG)
    if worst_mass_kg <= 0.0:
        return ATP_MIN_DECEL_MS2
    base_decel = equivalent_mass_adjusted_accel(force_n / worst_mass_kg) * brake_factor * ATP_ADHESION_FACTOR
    return max(ATP_MIN_DECEL_MS2, base_decel)


def gradient_adjusted_decel_ms2(
    base_decel_ms2: float,
    track_profile: List[Tuple[float, float, float, float]],
    start_pos_m: float,
    distance_m: float,
) -> float:
    worst_gradient = worst_gradient_in_range(track_profile, start_pos_m, distance_m)
    aiding_accel = max(0.0, -worst_gradient) * G
    return max(ATP_MIN_DECEL_MS2, base_decel_ms2 - aiding_accel)


def vital_delay_margin_m(speed_ms: float, delay_s: float) -> float:
    if delay_s <= 0.0:
        return 0.0
    return max(0.0, speed_ms) * delay_s


def next_lower_limit(
    track_profile: List[Tuple[float, float, float, float]],
    pos_m: float,
    current_psr: float,
    tsr_zones,
) -> Tuple[float, float]:
    best_dist = float("inf")
    best_speed = current_psr

    for start, _end, _grad, psr in track_profile:
        if start <= pos_m:
            continue
        if psr < current_psr:
            dist = start - pos_m
            if dist < best_dist or (dist == best_dist and psr < best_speed):
                best_dist = dist
                best_speed = psr

    for zone in tsr_zones:
        z_start = zone["start"]
        z_speed = zone["speed"]
        if z_start <= pos_m:
            continue
        if z_speed < current_psr:
            dist = z_start - pos_m
            if dist < best_dist or (dist == best_dist and z_speed < best_speed):
                best_dist = dist
                best_speed = z_speed

    if best_dist == float("inf"):
        return current_psr, float("inf")
    return best_speed, best_dist


def elevation_at(track_profile: List[Tuple[float, float, float, float]], pos_m: float) -> float:
    if pos_m <= track_profile[0][0]:
        start, _, gradient, _ = track_profile[0]
        return (pos_m - start) * gradient

    elev = 0.0
    for start, end, gradient, _ in track_profile:
        if pos_m >= end:
            elev += (end - start) * gradient
        else:
            elev += (pos_m - start) * gradient
            break
    return elev


class VitalBrakeModel:
    """ATP vital braking model with delay, build-up and gradient margins."""

    def __init__(
        self,
        track_profile: List[Tuple[float, float, float, float]],
        start_pos_m: float,
        position_uncertainty_m: float,
        vital_speed_ms: float,
        brake_build_s: float,
    ):
        self.track_profile = track_profile
        self.start_pos_m = start_pos_m
        self.position_uncertainty_m = position_uncertainty_m
        self.vital_speed_ms = vital_speed_ms
        self.brake_build_s = brake_build_s

    def adjusted_decel_ms2(self, base_decel_ms2: float, distance_m: float) -> float:
        return gradient_adjusted_decel_ms2(base_decel_ms2, self.track_profile, self.start_pos_m, distance_m)

    def usable_distance_m(self, distance_m: float, reaction_delay_s: float) -> float:
        delay_margin_m = vital_delay_margin_m(self.vital_speed_ms, reaction_delay_s)
        return max(0.0, distance_m - self.position_uncertainty_m - delay_margin_m)

    def speed_for_stop(self, distance_m: float, base_decel_ms2: float, reaction_delay_s: float) -> float:
        effective_decel_ms2 = self.adjusted_decel_ms2(base_decel_ms2, distance_m)
        return max_speed_with_buildup(
            self.usable_distance_m(distance_m, reaction_delay_s),
            effective_decel_ms2,
            self.brake_build_s,
        )

    def speed_for_target(
        self,
        target_speed_ms: float,
        distance_m: float,
        base_decel_ms2: float,
        reaction_delay_s: float,
    ) -> float:
        effective_decel_ms2 = self.adjusted_decel_ms2(base_decel_ms2, distance_m)
        return max_entry_speed_with_buildup(
            target_speed_ms,
            self.usable_distance_m(distance_m, reaction_delay_s),
            effective_decel_ms2,
            self.brake_build_s,
        )

    @staticmethod
    def supervised_location_m(mal_m: float) -> float:
        return mal_m + STOP_SVL_OFFSET_M


class AuthorityManager:
    """Wayside movement-authority calculator."""

    def __init__(self, track_end_m: float):
        self.track_end_m = track_end_m

    @staticmethod
    def _same_protection_line(train: object, front: object) -> bool:
        train_zone = getattr(train, "protection_zone_id", None)
        front_zone = getattr(front, "protection_zone_id", None)
        train_in_station = isinstance(train_zone, str) and train_zone.startswith("STATION:")
        front_in_station = isinstance(front_zone, str) and front_zone.startswith("STATION:")
        if train_in_station and front_in_station and train_zone != front_zone:
            stop = getattr(train, "active_scheduled_stop", None)
            if stop is not None:
                station_end = float(stop.get("pos_m", 0.0)) + float(stop.get("length_m", 160.0)) / 2.0
                if float(front.safe_rear_end_pos()) > station_end + 5.0:
                    return False
            return True
        if train_in_station != front_in_station:
            if train_in_station and not front_in_station:
                stop = getattr(train, "active_scheduled_stop", None)
                if stop is not None:
                    station_end = float(stop.get("pos_m", 0.0)) + float(stop.get("length_m", 160.0)) / 2.0
                    if float(front.safe_rear_end_pos()) > station_end + 5.0:
                        return False
            return True
        if train_zone is None or front_zone is None or train_zone != front_zone:
            return True
        return getattr(train, "protection_lane", 0) == getattr(front, "protection_lane", 0)

    def compute_mal(self, trains: List[object]) -> Dict[str, MovementAuthorityLimit]:
        active = sorted(trains, key=lambda item: item.reported_pos, reverse=True)
        mal_map: Dict[str, MovementAuthorityLimit] = {}
        for train in active:
            front = min(
                (
                    candidate
                    for candidate in active
                    if candidate.reported_pos > train.reported_pos
                    and self._same_protection_line(train, candidate)
                ),
                key=lambda candidate: candidate.reported_pos,
                default=None,
            )
            if front is None:
                mal_map[train.id] = MovementAuthorityLimit(
                    train_id=train.id,
                    mal_m=self.track_end_m,
                    protected_rear_m=self.track_end_m,
                    follower_braking_m=0.0,
                    follower_projection_m=0.0,
                    safety_margin_m=0.0,
                    overlap_m=0.0,
                    reason="LEADING_TRAIN_TRACK_END",
                )
                continue

            follower_uncertainty_m = train.effective_position_uncertainty_m()
            a_follow_emergency = conservative_brake_decel_ms2(
                EMERGENCY_FORCE_N,
                train.mass,
                ATP_EMERGENCY_BRAKE_FACTOR,
            )
            extrapolation_s = ATP_MA_EXTRAPOLATION_S + ATP_POS_REPORT_LATENCY_S + DT
            follower_vital_speed = max(train.vital_speed, train.speed)
            follower_max_accel = traction_acceleration_ms2(follower_vital_speed)
            follower_extrapolation = (
                follower_vital_speed * extrapolation_s
                + 0.5 * follower_max_accel * extrapolation_s * extrapolation_s
            )
            follower_braking = stopping_distance_with_buildup(
                follower_vital_speed,
                a_follow_emergency,
                ATP_BRAKE_BUILDUP_S,
            )
            follower_front_projection = train.reported_pos + follower_uncertainty_m + follower_extrapolation
            if front.trip_mode:
                point_to_protect = min(
                    getattr(front, "trip_protect_rear_pos", front.safe_rear_end_pos()),
                    front.safe_rear_end_pos(),
                )
                reason = "FRONT_TRAIN_TRIP_PROTECT"
            else:
                point_to_protect = front.safe_rear_end_pos()
                reason = "FRONT_TRAIN_SAFE_REAR"
            mal_map[train.id] = MovementAuthorityLimit(
                train_id=train.id,
                mal_m=point_to_protect - SAFETY_MARGIN_M - OVERLAP_M,
                protected_rear_m=point_to_protect,
                follower_braking_m=follower_braking,
                follower_projection_m=follower_front_projection - train.reported_pos,
                safety_margin_m=SAFETY_MARGIN_M,
                overlap_m=OVERLAP_M,
                reason=reason,
            )
        return mal_map

    def compute_eoa(self, trains: List[object]) -> Dict[str, float]:
        return {train_id: mal.mal_m for train_id, mal in self.compute_mal(trains).items()}


__all__ = [
    "ATP_ADHESION_FACTOR",
    "ATP_BRAKE_BUILDUP_S",
    "ATP_EMERGENCY_BRAKE_FACTOR",
    "ATP_MA_EXTRAPOLATION_S",
    "ATP_MIN_DECEL_MS2",
    "ATP_POS_REPORT_LATENCY_S",
    "AuthorityManager",
    "MovementAuthorityLimit",
    "SafeMovementPacket",
    "STOP_SVL_OFFSET_M",
    "VitalBrakeModel",
    "braking_curve_profile",
    "conservative_brake_decel_ms2",
    "get_track_info",
    "gradient_adjusted_decel_ms2",
    "max_entry_speed_with_buildup",
    "max_speed_with_buildup",
    "next_lower_limit",
    "stopping_distance_with_buildup",
    "vital_delay_margin_m",
    "worst_gradient_in_range",
    "elevation_at",
]
