from __future__ import annotations

from typing import Any, Dict, List, Tuple

from SUBSYSTEMS.signalling import AuthorityManager, MovementAuthorityLimit, SafeMovementPacket, get_track_info, next_lower_limit


class ZoneController:
    """Wayside/ZC logic that prepares safe packets from DCS-delivered reports."""

    def __init__(self, track_end_m: float):
        self.block_mode = "moving_block"
        self.authority_manager = AuthorityManager(track_end_m)
        self.last_valid_position_report: Dict[str, Dict[str, Any]] = {}
        self.position_report_freshness: Dict[str, str] = {}

    def store_position_report(self, train_id: str, report: Dict[str, Any], freshness: str = "FRESH"):
        self.last_valid_position_report[train_id] = dict(report)
        self.position_report_freshness[train_id] = freshness

    def mark_position_report_freshness(self, train_id: str, freshness: str):
        self.position_report_freshness[train_id] = freshness

    def compute_mal(self, trains: List[object]) -> Dict[str, MovementAuthorityLimit]:
        return self.authority_manager.compute_mal(trains)

    def compute_eoa(self, trains: List[object]) -> Dict[str, float]:
        return {train_id: mal.mal_m for train_id, mal in self.compute_mal(trains).items()}

    def build_safe_packets(
        self,
        track_profile: List[Tuple[float, float, float, float]],
        tsr_zones: List[Dict[str, float]],
        track_end_m: float,
        stop_eoa_map: Dict[str, float],
        trains_for_authority: List[object],
    ) -> Dict[str, SafeMovementPacket]:
        protection_trains = [
            train
            for train in trains_for_authority
            if bool(getattr(train, "has_position_report", True))
        ]
        if len(protection_trains) != len(trains_for_authority):
            return {}
        authority_trains = [
            train
            for train in protection_trains
            if bool(getattr(train, "may_receive_authority", False))
        ]
        mal_map = self.compute_mal(protection_trains)
        packets: Dict[str, SafeMovementPacket] = {}
        packet_order = sorted(authority_trains, key=lambda item: item.reported_pos, reverse=True)
        for train in packet_order:
            pos_for_limits = train.reported_pos
            if pos_for_limits < 0:
                gradient, psr = get_track_info(track_profile, track_profile[0][0])
            else:
                gradient, base_psr = get_track_info(track_profile, pos_for_limits)
                psr = base_psr
            for zone in tsr_zones:
                if zone["start"] <= pos_for_limits <= zone["end"]:
                    psr = min(psr, zone["speed"])
            next_speed, next_dist = next_lower_limit(track_profile, pos_for_limits, psr, tsr_zones)
            mal = mal_map.get(train.id)
            mal_m = mal.mal_m if mal is not None else track_end_m
            protected_by_obstacle = False
            if mal is not None:
                for obstacle in protection_trains:
                    if obstacle.id == train.id or bool(getattr(obstacle, "may_receive_authority", False)):
                        continue
                    if abs(float(obstacle.safe_rear_end_pos()) - float(mal.protected_rear_m)) <= 1.0:
                        protected_by_obstacle = True
                        break
            stop_eoa = stop_eoa_map.get(train.id)
            # Station stop/holding EOA is a constraint on top of moving-block MA,
            # not a replacement for leader protection on the open line.
            packet_eoa = min(mal_m, stop_eoa if stop_eoa is not None else track_end_m)
            packets[train.id] = SafeMovementPacket(
                eoa_m=packet_eoa,
                tsr_kmh=psr,
                variants={
                    "gradient": gradient,
                    "next_speed_limit_kmh": next_speed,
                    "next_speed_limit_dist_m": next_dist,
                    "ma_reason": "OBSTACLE_PROTECTION" if protected_by_obstacle else (mal.reason if mal is not None else "TRACK_END"),
                    "protected_rear_m": mal.protected_rear_m if mal is not None else track_end_m,
                },
            )
        return packets
