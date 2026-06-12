from __future__ import annotations

from collections import deque
from types import SimpleNamespace
from typing import Any

from SUBSYSTEMS.communication.messages import MovementAuthorityMessage
from SUBSYSTEMS.communication.rasta import VitalSafePacket, VitalSession
from SUBSYSTEMS.signalling import SafeMovementPacket


class DCSWatchdog:
    """Fail-safe communication watchdog for the onboard CC."""

    def __init__(self, timeout_s: float, startup_grace_s: float):
        self.timeout_s = timeout_s
        self.startup_grace_s = startup_grace_s
        self.last_receive_time_s = 0.0

    def mark_received(self, now_s: float):
        self.last_receive_time_s = now_s

    def age_s(self, now_s: float) -> float:
        return max(0.0, now_s - self.last_receive_time_s)

    def packet_is_valid(self, now_s: float, packet_integrity_ok: bool) -> bool:
        if not packet_integrity_ok:
            return False
        return self.age_s(now_s) <= self.timeout_s or now_s <= self.startup_grace_s


class OnboardControlCenter:
    """Train-local CC that only consumes safe packets from ZC/DCS."""

    def __init__(self, train_id: str, timeout_s: float, startup_grace_s: float):
        self.train_id = train_id
        self.latest_packet = SafeMovementPacket(
            eoa_m=0.0,
            tsr_kmh=25.0,
            variants={
                "gradient": 0.0,
                "next_speed_limit_kmh": 0.0,
                "next_speed_limit_dist_m": float("inf"),
            },
        )
        self.pending_packets: deque[tuple[float, SafeMovementPacket]] = deque()
        self.pending_vital_packets: deque[tuple[float, VitalSafePacket]] = deque()
        self.watchdog = DCSWatchdog(timeout_s, startup_grace_s)
        self.latest_packet_issued_time_s = -1.0
        self.vital_session = VitalSession(
            local_id=train_id,
            remote_id="ZC_01",
            session_id=f"ZC_01:{train_id}",
        )
        self.last_validation_result = "ACCEPTED"
        self.last_reject_reason = ""
        self.data_freshness = "FRESH"
        self.event_sink: Any = None

    def receive_safe_packet(self, packet: SafeMovementPacket, arrival_time_s: float):
        self.pending_packets.append((arrival_time_s, packet))
        self.pending_packets = deque(sorted(self.pending_packets, key=lambda item: item[0]))

    def receive_vital_packet(self, packet: VitalSafePacket, arrival_time_s: float):
        self.pending_vital_packets.append((arrival_time_s, packet))
        self.pending_vital_packets = deque(sorted(self.pending_vital_packets, key=lambda item: item[0]))

    def _packet_from_payload(self, payload: dict) -> SafeMovementPacket:
        return SafeMovementPacket(
            eoa_m=float(payload["eoa_m"]),
            tsr_kmh=float(payload["psr_kmh"]),
            variants={
                "gradient": float(payload.get("gradient", 0.0)),
                "next_speed_limit_kmh": float(payload.get("next_speed_limit_kmh", 0.0)),
                "next_speed_limit_dist_m": float(payload.get("next_speed_limit_dist_m", float("inf"))),
            },
            issued_time_s=float(payload.get("issued_time_s", 0.0)),
        )

    def _accept_vital_packet(self, packet: VitalSafePacket, now_s: float):
        result = self.vital_session.validate(packet, int(now_s * 1000))
        self.last_validation_result = result.result
        self.last_reject_reason = result.reason
        if self.event_sink is not None:
            self.event_sink.log_validation(now_s, packet, result.result, result.action, result.reason)
        if not result.accepted:
            return
        if packet.header.message_type != "MA_UPDATE":
            return
        safe_packet = self._packet_from_payload(packet.decoded_payload(self.vital_session.secret))
        if safe_packet.issued_time_s < self.latest_packet_issued_time_s:
            self.last_validation_result = "OUT_OF_ORDER"
            self.last_reject_reason = "internal issued_time older than latest accepted packet"
            return
        self.latest_packet = safe_packet
        self.latest_packet_issued_time_s = safe_packet.issued_time_s
        self.watchdog.mark_received(now_s)
        self.data_freshness = "FRESH"

    def apply_to_train(self, train: object, now_s: float):
        while self.pending_vital_packets and self.pending_vital_packets[0][0] <= now_s:
            _, packet = self.pending_vital_packets.popleft()
            self._accept_vital_packet(packet, now_s)
        while self.pending_packets and self.pending_packets[0][0] <= now_s:
            _, packet = self.pending_packets.popleft()
            if packet.issued_time_s < self.latest_packet_issued_time_s:
                continue
            self.latest_packet = packet
            self.latest_packet_issued_time_s = packet.issued_time_s
            self.watchdog.mark_received(now_s)
        packet = self.latest_packet
        train.eoa = packet.eoa_m
        train.psr_kmh = min(packet.tsr_kmh, 25.0) if train.drive_mode == "CMD25" else packet.tsr_kmh
        train.gradient = float(packet.variants.get("gradient", 0.0))
        train.limit_ahead_speed_kmh = float(packet.variants.get("next_speed_limit_kmh", 0.0))
        train.limit_ahead_dist = float(packet.variants.get("next_speed_limit_dist_m", float("inf")))
        packet_valid = (
            packet.tsr_kmh >= 0.0
            and train.limit_ahead_dist >= 0.0
            and not (packet.eoa_m != packet.eoa_m)
            and not (train.gradient != train.gradient)
        )
        train.safe_packet_age_s = self.watchdog.age_s(now_s)
        train.safe_packet_valid = self.watchdog.packet_is_valid(now_s, packet_valid)
        if not train.safe_packet_valid:
            self.data_freshness = "LOST" if train.safe_packet_age_s > self.watchdog.timeout_s else "EXPIRED"
        elif train.safe_packet_age_s > max(0.0, self.watchdog.timeout_s * 0.5):
            self.data_freshness = "STALE"
        else:
            self.data_freshness = "FRESH"
        train.vital_packet_result = self.last_validation_result
        train.vital_packet_reason = self.last_reject_reason
        train.ma_freshness = self.data_freshness


class RedundantOnboardControlCenter:
    """Dual onboard CC channels with hot-standby vital packet voting."""

    def __init__(self, train_id: str, timeout_s: float, startup_grace_s: float):
        self.train_id = train_id
        self.primary = OnboardControlCenter(f"{train_id}:CC_A", timeout_s, startup_grace_s)
        self.secondary = OnboardControlCenter(f"{train_id}:CC_B", timeout_s, startup_grace_s)
        for channel in self.channels:
            channel.vital_session.local_id = train_id
            channel.vital_session.session_id = f"ZC_01:{train_id}"
        self.vital_session = self.primary.vital_session
        self.watchdog = self.primary.watchdog
        self.active_channel = "CC_A"
        self.redundancy_state = "DUAL_ACTIVE"
        self._event_sink: Any = None
        self.pending_packets = _RedundantQueueProxy(self.channels, "pending_packets")
        self.pending_vital_packets = _RedundantQueueProxy(self.channels, "pending_vital_packets")

    @property
    def channels(self):
        return (self.primary, self.secondary)

    @property
    def event_sink(self) -> Any:
        return self._event_sink

    @event_sink.setter
    def event_sink(self, value: Any):
        self._event_sink = value
        for channel in self.channels:
            channel.event_sink = value

    def receive_safe_packet(self, packet: SafeMovementPacket, arrival_time_s: float):
        for channel in self.channels:
            channel.receive_safe_packet(packet, arrival_time_s)

    def receive_vital_packet(self, packet: VitalSafePacket, arrival_time_s: float):
        for channel in self.channels:
            channel.receive_vital_packet(packet, arrival_time_s)

    def mark_received(self, now_s: float):
        for channel in self.channels:
            channel.watchdog.mark_received(now_s)

    def _apply_channel_to_shadow(self, channel: OnboardControlCenter, train: object, now_s: float):
        shadow = SimpleNamespace(
            drive_mode=getattr(train, "drive_mode", "ATO"),
            eoa=getattr(train, "eoa", 0.0),
            psr_kmh=getattr(train, "psr_kmh", 25.0),
            gradient=getattr(train, "gradient", 0.0),
            limit_ahead_speed_kmh=getattr(train, "limit_ahead_speed_kmh", 0.0),
            limit_ahead_dist=getattr(train, "limit_ahead_dist", float("inf")),
            safe_packet_age_s=getattr(train, "safe_packet_age_s", 0.0),
            safe_packet_valid=getattr(train, "safe_packet_valid", False),
            vital_packet_result=getattr(train, "vital_packet_result", ""),
            vital_packet_reason=getattr(train, "vital_packet_reason", ""),
            ma_freshness=getattr(train, "ma_freshness", "LOST"),
        )
        channel.apply_to_train(shadow, now_s)
        return shadow

    def _select_channel(self, left: tuple[str, OnboardControlCenter, Any], right: tuple[str, OnboardControlCenter, Any]):
        candidates = [left, right]
        candidates.sort(
            key=lambda item: (
                bool(item[2].safe_packet_valid),
                -float(item[2].safe_packet_age_s),
                float(item[1].latest_packet_issued_time_s),
            ),
            reverse=True,
        )
        return candidates[0]

    def apply_to_train(self, train: object, now_s: float):
        left = ("CC_A", self.primary, self._apply_channel_to_shadow(self.primary, train, now_s))
        right = ("CC_B", self.secondary, self._apply_channel_to_shadow(self.secondary, train, now_s))
        active_name, active_channel, shadow = self._select_channel(left, right)
        train.eoa = shadow.eoa
        train.psr_kmh = shadow.psr_kmh
        train.gradient = shadow.gradient
        train.limit_ahead_speed_kmh = shadow.limit_ahead_speed_kmh
        train.limit_ahead_dist = shadow.limit_ahead_dist
        train.safe_packet_age_s = shadow.safe_packet_age_s
        train.safe_packet_valid = shadow.safe_packet_valid
        train.vital_packet_result = shadow.vital_packet_result
        train.vital_packet_reason = shadow.vital_packet_reason
        train.ma_freshness = shadow.ma_freshness
        self.active_channel = active_name
        self.watchdog = active_channel.watchdog
        valid_count = sum(1 for _name, _channel, item in (left, right) if item.safe_packet_valid)
        self.redundancy_state = "DUAL_ACTIVE" if valid_count == 2 else "DEGRADED" if valid_count == 1 else "LOST"


class _RedundantQueueProxy:
    def __init__(self, channels: tuple[OnboardControlCenter, OnboardControlCenter], attr: str):
        self._channels = channels
        self._attr = attr

    def clear(self):
        for channel in self._channels:
            getattr(channel, self._attr).clear()

    def __len__(self):
        return max((len(getattr(channel, self._attr)) for channel in self._channels), default=0)


__all__ = [
    "DCSWatchdog",
    "OnboardControlCenter",
    "RedundantOnboardControlCenter",
    "SafeMovementPacket",
    "MovementAuthorityMessage",
]
