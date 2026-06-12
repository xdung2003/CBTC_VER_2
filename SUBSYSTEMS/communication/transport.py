from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Deque, Dict, List, Optional, Tuple


class DcsPathState(str, Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


@dataclass
class DcsNetworkPath:
    name: str
    state: DcsPathState = DcsPathState.OK
    base_latency_ms: float = 70.0
    jitter_ms: float = 20.0
    packet_loss_rate: float = 0.0
    sent_count: int = 0
    accepted_count: int = 0
    lost_count: int = 0
    timeout_count: int = 0

    def latency_ms(self, edge_factor: float = 0.0, high_latency: bool = False) -> float:
        multiplier = 1.0 + edge_factor * 2.5
        if self.state == DcsPathState.DEGRADED:
            multiplier *= 2.0
        if high_latency:
            multiplier *= 4.0
        jitter = random.uniform(0.0, self.jitter_ms * multiplier)
        return max(0.0, self.base_latency_ms * multiplier + jitter)

    def loss_rate(self, edge_factor: float = 0.0, packet_loss_fault: bool = False) -> float:
        if self.state == DcsPathState.LOST:
            return 1.0
        rate = self.packet_loss_rate + edge_factor * 0.25
        if self.state == DcsPathState.DEGRADED:
            rate += 0.15
        if packet_loss_fault:
            rate += 0.45
        return min(1.0, max(0.0, rate))


@dataclass(frozen=True)
class RadioAccessPoint:
    id: str
    start_m: float
    end_m: float

    def contains(self, pos_m: float) -> bool:
        return self.start_m <= pos_m <= self.end_m

    def edge_factor(self, pos_m: float, edge_width_m: float = 80.0) -> float:
        if not self.contains(pos_m):
            return 1.0
        dist_to_edge = min(abs(pos_m - self.start_m), abs(self.end_m - pos_m))
        return max(0.0, min(1.0, 1.0 - dist_to_edge / max(1.0, edge_width_m)))


@dataclass
class DcsPacketEvent:
    time_s: float
    source_id: str
    destination_id: str
    protocol: str
    path: str
    msg_type: str
    sequence_number: int
    latency_ms: float
    ttl_state: str
    result: str
    action: str
    reason: str
    details: Dict[str, Any] | None = None


class DcsTransport:
    def __init__(self, radio_access_points: Optional[List[Dict[str, Any]]] = None, radio_physical: Optional[Dict[str, Any]] = None):
        self.paths: Dict[str, DcsNetworkPath] = {
            "RED": DcsNetworkPath("RED", base_latency_ms=70.0, jitter_ms=18.0),
            "BLUE": DcsNetworkPath("BLUE", base_latency_ms=80.0, jitter_ms=22.0),
        }
        self.active_path = "RED"
        self.events: Deque[DcsPacketEvent] = deque(maxlen=240)
        self.last_rap_by_train: Dict[str, str] = {}
        self.last_fault: str = ""
        self.handover_count: int = 0
        self.last_vital_packets: Dict[Tuple[str, str, str], Any] = {}
        self.faults: Dict[str, bool] = {
            "radio_coverage_loss": False,
            "handover_failure": False,
            "high_latency": False,
            "packet_loss": False,
            "crc_corruption": False,
            "hmac_corruption": False,
            "replay_attack": False,
            "out_of_order_packet": False,
            "opcua_loss": False,
            "ber_corruption": False,
        }
        self.radio_physical = {
            "enabled": True,
            "modulation": "OFDM-QAM-like",
            "signal_quality_model": "abstract",
        }
        if radio_physical:
            self.radio_physical.update(radio_physical)
        self.radio_access_points = [
            RadioAccessPoint(str(item["id"]), float(item["start_m"]), float(item["end_m"]))
            for item in (radio_access_points or self.default_radio_access_points())
        ]

    @staticmethod
    def default_radio_access_points() -> List[Dict[str, Any]]:
        return [
            {"id": "RAP_01", "start_m": -300.0, "end_m": 900.0},
            {"id": "RAP_02", "start_m": 800.0, "end_m": 1700.0},
            {"id": "RAP_03", "start_m": 1600.0, "end_m": 2600.0},
            {"id": "RAP_04", "start_m": 2500.0, "end_m": 4200.0},
        ]

    def set_path_state(self, path: str, state: str):
        key = path.upper()
        if key in self.paths:
            self.paths[key].state = DcsPathState(state.upper())
            self.last_fault = f"{key} path {self.paths[key].state.value}"

    def set_fault(self, fault: str, active: bool):
        if fault in self.faults:
            self.faults[fault] = bool(active)
            self.last_fault = f"{fault} {'ON' if active else 'OFF'}"

    def coverage_for_position(self, pos_m: float) -> Tuple[Optional[RadioAccessPoint], float, bool]:
        covering = [rap for rap in self.radio_access_points if rap.contains(pos_m)]
        if not covering or self.faults["radio_coverage_loss"]:
            return None, 1.0, False
        rap = min(covering, key=lambda item: item.edge_factor(pos_m))
        overlap = len(covering) > 1
        return rap, rap.edge_factor(pos_m), overlap

    def _modulation_sample(self, rap: Optional[RadioAccessPoint], edge_factor: float) -> Dict[str, Any]:
        if not self.radio_physical.get("enabled", True):
            return {
                "modulation_scheme": "DISABLED",
                "signal_quality": "GOOD",
                "bit_error_rate_sim": 0.0,
                "symbol_error_sim": 0.0,
                "coverage_status": "NOT_MODELED",
                "handover_status": "N/A",
            }
        if rap is None:
            return {
                "modulation_scheme": self.radio_physical.get("modulation", "OFDM-QAM-like"),
                "signal_quality": "LOST",
                "bit_error_rate_sim": 1.0,
                "symbol_error_sim": 1.0,
                "coverage_status": "OUT_OF_COVERAGE",
                "handover_status": "NONE",
            }
        weak = edge_factor > 0.55
        ber = 0.00001 + edge_factor * 0.0025
        ser = 0.00005 + edge_factor * 0.01
        return {
            "modulation_scheme": self.radio_physical.get("modulation", "OFDM-QAM-like"),
            "signal_quality": "WEAK" if weak else "GOOD",
            "bit_error_rate_sim": ber,
            "symbol_error_sim": ser,
            "coverage_status": "EDGE" if weak else "IN_COVERAGE",
            "handover_status": "OVERLAP_CANDIDATE",
        }

    def _choose_path(self) -> Tuple[Optional[DcsNetworkPath], bool]:
        active = self.paths[self.active_path]
        if active.state != DcsPathState.LOST:
            return active, False
        alternate_name = "BLUE" if self.active_path == "RED" else "RED"
        alternate = self.paths[alternate_name]
        if alternate.state != DcsPathState.LOST:
            self.active_path = alternate_name
            return alternate, True
        return None, False

    def transport_vital(self, packet: Any, now_s: float, train_id: str, train_pos_m: float) -> Tuple[Optional[Any], float, DcsPacketEvent]:
        rap, edge_factor, overlap = self.coverage_for_position(train_pos_m)
        path, failover = self._choose_path()
        seq = int(getattr(packet.header, "sequence_number", -1))
        msg_type = str(getattr(packet.header, "message_type", ""))
        source = str(getattr(packet.header, "source_id", ""))
        dest = str(getattr(packet.header, "destination_id", ""))
        if path is None:
            for item in self.paths.values():
                item.timeout_count += 1
            event = self._event(now_s, source, dest, "RASTA_VITAL", "NONE", msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "both RED and BLUE lost")
            return None, now_s, event
        path.sent_count += 1
        path_label = path.name + (f"+{rap.id}" if rap is not None else "")
        if failover:
            self._event(now_s, source, dest, "RASTA_VITAL", path.name, msg_type, seq, 0.0, "OK", "FAILOVER", "path switched", f"active path -> {path.name}")
        if rap is None:
            path.lost_count += 1
            path.timeout_count += 1
            modulation = self._modulation_sample(None, edge_factor)
            return None, now_s, self._event(now_s, source, dest, "RASTA_VITAL", path_label, msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "outside radio coverage", details={"radio": modulation})
        last_rap = self.last_rap_by_train.get(train_id)
        if last_rap and last_rap != rap.id:
            self.handover_count += 1
            self._event(now_s, source, dest, "RASTA_VITAL", path_label, "HANDOVER", seq, 0.0, "OK", "ACCEPTED", "handover", f"{last_rap}->{rap.id}", details={"handover": f"{last_rap}->{rap.id}"})
            if self.faults["handover_failure"]:
                path.lost_count += 1
                path.timeout_count += 1
                return None, now_s, self._event(now_s, source, dest, "RASTA_VITAL", path_label, msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "RAP handover failure")
        self.last_rap_by_train[train_id] = rap.id
        loss_rate = path.loss_rate(edge_factor, self.faults["packet_loss"])
        if random.random() < loss_rate:
            path.lost_count += 1
            path.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "RASTA_VITAL", path_label, msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "packet loss")
        latency = path.latency_ms(edge_factor, self.faults["high_latency"])
        delivered = packet
        modulation = self._modulation_sample(rap, edge_factor)
        replay_key = (source, dest, msg_type)
        if self.faults["replay_attack"] and replay_key in self.last_vital_packets:
            delivered = self.last_vital_packets[replay_key]
        elif self.faults["out_of_order_packet"] and hasattr(packet, "with_sequence_number"):
            delivered = packet.with_sequence_number(max(0, seq - 2), "cbtc-sim-shared-secret", new_uuid=True)
        if self.faults["crc_corruption"] and hasattr(packet, "with_crc_corruption"):
            delivered = delivered.with_crc_corruption()
        if self.faults["hmac_corruption"] and hasattr(delivered, "with_hmac_corruption"):
            delivered = delivered.with_hmac_corruption()
        if (
            self.faults["ber_corruption"]
            or random.random() < min(0.2, float(modulation["bit_error_rate_sim"]) * 10.0)
        ) and hasattr(delivered, "with_encrypted_payload_corruption"):
            delivered = delivered.with_encrypted_payload_corruption()
        if not self.faults["replay_attack"]:
            self.last_vital_packets[replay_key] = packet
        path.accepted_count += 1
        delivered_header = getattr(delivered, "header", None)
        delivered_safety = getattr(delivered, "safety", None)
        details = {
            "route": {"source": source, "destination": dest, "protocol": "RASTA_VITAL", "path": path.name, "rap": rap.id},
            "message": {"type": msg_type, "schema": msg_type, "payload": getattr(packet, "payload", {})},
            "frame": {
                "sent_header": getattr(packet, "header", None).__dict__ if hasattr(getattr(packet, "header", None), "__dict__") else {},
                "received_header": delivered_header.__dict__ if hasattr(delivered_header, "__dict__") else {},
                "safety": delivered_safety.__dict__ if hasattr(delivered_safety, "__dict__") else {},
                "encryption_enabled": getattr(delivered, "encryption_enabled", False),
                "encryption_algorithm": getattr(delivered, "encryption_algorithm", ""),
                "encrypted_payload": getattr(delivered, "encrypted_payload", ""),
                "payload_format": getattr(delivered, "payload_format", ""),
            },
            "radio": modulation,
            "chain": [
                "Message",
                "Serialized Payload",
                "Encrypted Payload",
                "VitalSafePacket",
                "DCS Transport",
                "Decrypt",
                "Decoded Message",
            ],
        }
        return delivered, now_s + latency / 1000.0, self._event(now_s, source, dest, "RASTA_VITAL", path_label, msg_type, seq, latency, "OK", "DELIVERED", "queued", "transported", details=details)

    def transport_vital_wired(self, packet: Any, now_s: float) -> Tuple[Optional[Any], float, DcsPacketEvent]:
        path, failover = self._choose_path()
        seq = int(getattr(packet.header, "sequence_number", -1))
        msg_type = str(getattr(packet.header, "message_type", ""))
        source = str(getattr(packet.header, "source_id", ""))
        dest = str(getattr(packet.header, "destination_id", ""))
        if path is None:
            for item in self.paths.values():
                item.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "RASTA_VITAL", "NONE", msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "wired backbone lost")
        path.sent_count += 1
        if failover:
            self._event(now_s, source, dest, "RASTA_VITAL", path.name, msg_type, seq, 0.0, "OK", "FAILOVER", "path switched", f"active path -> {path.name}")
        if random.random() < path.loss_rate(0.0, self.faults["packet_loss"]) * 0.25:
            path.lost_count += 1
            path.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "RASTA_VITAL", path.name, msg_type, seq, 0.0, "OK", "TIMEOUT", "ignored", "wired packet loss")
        delivered = packet
        replay_key = (source, dest, msg_type)
        if self.faults["replay_attack"] and replay_key in self.last_vital_packets:
            delivered = self.last_vital_packets[replay_key]
        elif self.faults["out_of_order_packet"] and hasattr(packet, "with_sequence_number"):
            delivered = packet.with_sequence_number(max(0, seq - 2), "cbtc-sim-shared-secret", new_uuid=True)
        if self.faults["crc_corruption"] and hasattr(packet, "with_crc_corruption"):
            delivered = delivered.with_crc_corruption()
        if self.faults["hmac_corruption"] and hasattr(delivered, "with_hmac_corruption"):
            delivered = delivered.with_hmac_corruption()
        if not self.faults["replay_attack"]:
            self.last_vital_packets[replay_key] = packet
        latency = path.latency_ms(0.0, self.faults["high_latency"]) * 0.35
        path.accepted_count += 1
        return delivered, now_s + latency / 1000.0, self._event(now_s, source, dest, "RASTA_VITAL", path.name, msg_type, seq, latency, "OK", "DELIVERED", "queued", "wired backbone", details={"route": {"source": source, "destination": dest, "protocol": "RASTA_VITAL", "path": path.name, "rap": "wired/backbone"}})

    def transport_train_status_supervision(self, frame: Any, now_s: float, train_id: str, train_pos_m: float) -> Tuple[Optional[Any], float, DcsPacketEvent]:
        rap, edge_factor, _overlap = self.coverage_for_position(train_pos_m)
        path, failover = self._choose_path()
        source = str(getattr(frame, "source_id", ""))
        dest = str(getattr(frame, "destination_id", ""))
        method = str(getattr(frame, "method_name", ""))
        retry = int(getattr(frame, "retry_count", 0))
        if path is None:
            for item in self.paths.values():
                item.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "OPCUA_SUPERVISION", "NONE", method, retry, 0.0, "OK", "TIMEOUT", "ignored", "supervision radio backbone lost")
        path.sent_count += 1
        path_label = path.name + (f"+{rap.id}" if rap is not None else "")
        if failover:
            self._event(now_s, source, dest, "OPCUA_SUPERVISION", path.name, method, retry, 0.0, "OK", "FAILOVER", "path switched", f"active path -> {path.name}")
        if self.faults["opcua_loss"]:
            path.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, method, retry, 0.0, "OK", "TIMEOUT", "ignored", "OPC UA-like supervision loss")
        modulation = self._modulation_sample(rap, edge_factor)
        if rap is None:
            path.lost_count += 1
            path.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, method, retry, 0.0, "OK", "TIMEOUT", "ignored", "outside radio coverage", details={"radio": modulation})
        last_rap = self.last_rap_by_train.get(train_id)
        if last_rap and last_rap != rap.id:
            self.handover_count += 1
            self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, "HANDOVER", retry, 0.0, "OK", "ACCEPTED", "handover", f"{last_rap}->{rap.id}", details={"handover": f"{last_rap}->{rap.id}"})
            if self.faults["handover_failure"]:
                path.lost_count += 1
                path.timeout_count += 1
                return None, now_s, self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, method, retry, 0.0, "OK", "TIMEOUT", "ignored", "RAP handover failure")
        self.last_rap_by_train[train_id] = rap.id
        if random.random() < path.loss_rate(edge_factor, self.faults["packet_loss"]):
            path.lost_count += 1
            path.timeout_count += 1
            return None, now_s, self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, method, retry, 0.0, "OK", "TIMEOUT", "ignored", "supervision radio packet loss")
        latency = path.latency_ms(edge_factor, self.faults["high_latency"])
        path.accepted_count += 1
        details = {
            "route": {"source": source, "destination": dest, "protocol": "OPCUA_SUPERVISION", "path": path.name, "rap": rap.id},
            "message": {"type": method, "schema": "OpcUaSupervisionFrame", "payload": getattr(frame, "payload", {})},
            "frame": {
                "request_id": getattr(frame, "request_id", ""),
                "response_id": getattr(frame, "response_id", ""),
                "timestamp_ms": getattr(frame, "timestamp_ms", 0),
                "timeout_ms": getattr(frame, "timeout_ms", 0),
                "encryption_enabled": getattr(frame, "encryption_enabled", False),
                "encryption_algorithm": getattr(frame, "encryption_algorithm", ""),
                "key_id": getattr(frame, "key_id", ""),
                "encrypted_payload": getattr(frame, "encrypted_payload", ""),
                "payload_format": getattr(frame, "payload_format", ""),
            },
            "radio": modulation,
            "chain": ["Message", "Serialized Payload", "Encrypted Payload", "OPC UA-like Frame", "DCS Radio Transport", "Decrypt", "Decoded Message"],
        }
        return frame, now_s + latency / 1000.0, self._event(now_s, source, dest, "OPCUA_SUPERVISION", path_label, method, retry, latency, "OK", "ACCEPTED", "radio supervision delivered", "train radio", details=details)

    def transport_supervision(self, frame: Any, now_s: float) -> Tuple[Optional[Any], float, DcsPacketEvent]:
        path, failover = self._choose_path()
        source = str(getattr(frame, "source_id", ""))
        dest = str(getattr(frame, "destination_id", ""))
        method = str(getattr(frame, "method_name", ""))

        if hasattr(frame, "is_vital_forbidden") and frame.is_vital_forbidden():
            return None, now_s, self._event(
                now_s, source, dest, "OPCUA_SUPERVISION", "NONE",
                method, int(getattr(frame, "retry_count", 0)),
                0.0, "OK", "REJECTED", "ignored",
                "OPC UA-like cannot carry MA/EOA",
            )

        if path is None:
            for item in self.paths.values():
                item.timeout_count += 1
            return None, now_s, self._event(
                now_s, source, dest, "OPCUA_SUPERVISION", "NONE",
                method, int(getattr(frame, "retry_count", 0)),
                0.0, "OK", "TIMEOUT", "ignored",
                "supervision network lost",
            )

        path.sent_count += 1

        if self.faults["opcua_loss"]:
            path.timeout_count += 1
            return None, now_s, self._event(
                now_s, source, dest, "OPCUA_SUPERVISION", path.name,
                method, int(getattr(frame, "retry_count", 0)),
                0.0, "OK", "TIMEOUT", "ignored",
                "OPC UA-like supervision loss",
            )

        if failover:
            self._event(
                now_s, source, dest, "OPCUA_SUPERVISION", path.name,
                method, 0, 0.0, "OK", "FAILOVER", "path switched",
                f"active path -> {path.name}",
            )

        loss_rate = path.loss_rate(0.0, self.faults["packet_loss"]) * 0.5
        if random.random() < loss_rate:
            path.lost_count += 1
            path.timeout_count += 1
            return None, now_s, self._event(
                now_s, source, dest, "OPCUA_SUPERVISION", path.name,
                method, int(getattr(frame, "retry_count", 0)),
                0.0, "OK", "TIMEOUT", "ignored",
                "supervision packet loss",
            )

        latency = path.latency_ms(0.0, self.faults["high_latency"]) * 0.45
        path.accepted_count += 1

        details = {
            "route": {
                "source": source,
                "destination": dest,
                "protocol": "OPCUA_SUPERVISION",
                "path": path.name,
                "rap": "wired/supervision",
            },
            "message": {
                "type": method,
                "schema": "OpcUaSupervisionFrame",
                "payload": getattr(frame, "payload", {}),
            },
            "frame": {
                "request_id": getattr(frame, "request_id", ""),
                "response_id": getattr(frame, "response_id", ""),
                "timestamp_ms": getattr(frame, "timestamp_ms", 0),
                "timeout_ms": getattr(frame, "timeout_ms", 0),
                "encryption_enabled": getattr(frame, "encryption_enabled", False),
                "encryption_algorithm": getattr(frame, "encryption_algorithm", ""),
                "key_id": getattr(frame, "key_id", ""),
                "encrypted_payload": getattr(frame, "encrypted_payload", ""),
                "payload_format": getattr(frame, "payload_format", ""),
            },
            "radio": {
                "modulation_scheme": "N/A",
                "signal_quality": "GOOD",
                "bit_error_rate_sim": 0.0,
                "symbol_error_sim": 0.0,
                "coverage_status": "SUPERVISION_NETWORK",
                "handover_status": "N/A",
            },
            "chain": [
                "Message",
                "Serialized Payload",
                "Encrypted Payload",
                "OPC UA-like Frame",
                "DCS Transport",
                "Decrypt",
                "Decoded Message",
            ],
        }

        return frame, now_s + latency / 1000.0, self._event(
            now_s, source, dest, "OPCUA_SUPERVISION", path.name,
            method, int(getattr(frame, "retry_count", 0)),
            latency, "OK", "ACCEPTED", "supervision delivered",
            "non-vital", details=details,
        )
    def log_validation(self, now_s: float, packet: Any, result: str, action: str, reason: str, latency_ms: float = 0.0, path: str = "CC"):
        self._event(
            now_s,
            str(getattr(packet.header, "source_id", "")),
            str(getattr(packet.header, "destination_id", "")),
            "RASTA_VITAL",
            path,
            str(getattr(packet.header, "message_type", "")),
            int(getattr(packet.header, "sequence_number", -1)),
            latency_ms,
            "OK",
            result,
            action,
            reason,
        )

    def _event(self, time_s: float, source: str, dest: str, protocol: str, path: str, msg_type: str, seq: int, latency: float, ttl: str, result: str, action: str, reason: str, details: Dict[str, Any] | None = None) -> DcsPacketEvent:
        event = DcsPacketEvent(time_s, source, dest, protocol, path, msg_type, seq, latency, ttl, result, action, reason, details)
        self.events.append(event)
        return event
