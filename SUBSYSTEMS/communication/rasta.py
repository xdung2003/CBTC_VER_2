from __future__ import annotations

import hashlib
import hmac
import json
import math
import base64
import uuid
import zlib
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Set


class RastaSessionState(str, Enum):
    CONNECTING = "CONNECTING"
    ESTABLISHED = "ESTABLISHED"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


def canonical_payload(payload: Dict[str, Any]) -> bytes:
    return json.dumps(_json_safe(payload), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        if value > 0:
            return "Infinity"
        if value < 0:
            return "-Infinity"
        return "NaN"
    return value


def _xor_stream(data: bytes, secret: str, key_id: str) -> bytes:
    seed = hashlib.sha256(f"{key_id}:{secret}".encode("utf-8")).digest()
    out = bytearray()
    counter = 0
    while len(out) < len(data):
        out.extend(hashlib.sha256(seed + counter.to_bytes(4, "big")).digest())
        counter += 1
    return bytes(byte ^ mask for byte, mask in zip(data, out))


def encrypt_payload_bytes(payload_bytes: bytes, secret: str, key_id: str) -> str:
    protected = b"CBTC_SIM_PAYLOAD_V1:" + payload_bytes
    return base64.b64encode(_xor_stream(protected, secret, key_id)).decode("ascii")


def decrypt_payload_bytes(encrypted_payload: str, secret: str, key_id: str) -> bytes:
    try:
        cipher = base64.b64decode(encrypted_payload.encode("ascii"), validate=True)
    except Exception as exc:
        raise ValueError("encrypted payload is not valid base64") from exc
    protected = _xor_stream(cipher, secret, key_id)
    prefix = b"CBTC_SIM_PAYLOAD_V1:"
    if not protected.startswith(prefix):
        raise ValueError("decryption authentication marker mismatch")
    return protected[len(prefix):]


@dataclass(frozen=True)
class VitalPacketHeader:
    source_id: str
    destination_id: str
    session_id: str
    message_type: str
    sequence_number: int
    timestamp_ms: int
    ttl_ms: int
    payload_length: int


@dataclass(frozen=True)
class VitalPacketSafety:
    packet_uuid: str
    crc32: int
    hmac_sha256: str
    key_id: str


@dataclass(frozen=True)
class VitalSafePacket:
    header: VitalPacketHeader
    safety: VitalPacketSafety
    payload: Dict[str, Any]
    encryption_enabled: bool = True
    encryption_algorithm: str = "SIM_AES"
    encrypted_payload: str = ""
    payload_format: str = "json"

    def decoded_payload(self, secret: str) -> Dict[str, Any]:
        if self.encryption_enabled:
            payload_bytes = decrypt_payload_bytes(self.encrypted_payload, secret, self.safety.key_id)
            decoded = json.loads(payload_bytes.decode("utf-8"))
            if not isinstance(decoded, dict):
                raise ValueError("decoded payload is not an object")
            return decoded
        return dict(self.payload)

    @classmethod
    def create(
        cls,
        *,
        source_id: str,
        destination_id: str,
        session_id: str,
        message_type: str,
        sequence_number: int,
        timestamp_ms: int,
        ttl_ms: int,
        payload: Dict[str, Any],
        key_id: str,
        secret: str,
        encryption_enabled: bool = True,
    ) -> "VitalSafePacket":
        payload_bytes = canonical_payload(payload)
        protected_payload_bytes = (
            encrypt_payload_bytes(payload_bytes, secret, key_id).encode("ascii")
            if encryption_enabled
            else payload_bytes
        )
        header = VitalPacketHeader(
            source_id=source_id,
            destination_id=destination_id,
            session_id=session_id,
            message_type=message_type,
            sequence_number=int(sequence_number),
            timestamp_ms=int(timestamp_ms),
            ttl_ms=int(ttl_ms),
            payload_length=len(protected_payload_bytes),
        )
        packet_uuid = str(uuid.uuid4())
        crc = zlib.crc32(protected_payload_bytes) & 0xFFFFFFFF
        mac = cls._hmac(header, packet_uuid, key_id, protected_payload_bytes, secret)
        return cls(
            header=header,
            safety=VitalPacketSafety(packet_uuid=packet_uuid, crc32=crc, hmac_sha256=mac, key_id=key_id),
            payload=dict(payload),
            encryption_enabled=encryption_enabled,
            encryption_algorithm="SIM_AES" if encryption_enabled else "NONE",
            encrypted_payload=protected_payload_bytes.decode("ascii") if encryption_enabled else "",
            payload_format="json",
        )

    @staticmethod
    def _hmac(header: VitalPacketHeader, packet_uuid: str, key_id: str, payload_bytes: bytes, secret: str) -> str:
        header_bytes = json.dumps(header.__dict__, sort_keys=True, separators=(",", ":")).encode("utf-8")
        body = header_bytes + b"." + packet_uuid.encode("ascii") + b"." + key_id.encode("ascii") + b"." + payload_bytes
        return hmac.new(secret.encode("utf-8"), body, hashlib.sha256).hexdigest()

    def with_crc_corruption(self) -> "VitalSafePacket":
        return VitalSafePacket(
            header=self.header,
            safety=VitalPacketSafety(
                packet_uuid=self.safety.packet_uuid,
                crc32=(self.safety.crc32 ^ 0xFFFFFFFF) & 0xFFFFFFFF,
                hmac_sha256=self.safety.hmac_sha256,
                key_id=self.safety.key_id,
            ),
            payload=dict(self.payload),
            encryption_enabled=self.encryption_enabled,
            encryption_algorithm=self.encryption_algorithm,
            encrypted_payload=self.encrypted_payload,
            payload_format=self.payload_format,
        )

    def with_hmac_corruption(self) -> "VitalSafePacket":
        return VitalSafePacket(
            header=self.header,
            safety=VitalPacketSafety(
                packet_uuid=self.safety.packet_uuid,
                crc32=self.safety.crc32,
                hmac_sha256="0" * 64,
                key_id=self.safety.key_id,
            ),
            payload=dict(self.payload),
            encryption_enabled=self.encryption_enabled,
            encryption_algorithm=self.encryption_algorithm,
            encrypted_payload=self.encrypted_payload,
            payload_format=self.payload_format,
        )

    def with_encrypted_payload_corruption(self) -> "VitalSafePacket":
        corrupted = (self.encrypted_payload[:-2] + "AA") if self.encrypted_payload else ""
        return VitalSafePacket(
            header=self.header,
            safety=self.safety,
            payload=dict(self.payload),
            encryption_enabled=self.encryption_enabled,
            encryption_algorithm=self.encryption_algorithm,
            encrypted_payload=corrupted,
            payload_format=self.payload_format,
        )

    def with_decrypt_error(self, secret: str) -> "VitalSafePacket":
        corrupted = encrypt_payload_bytes(b"not-json-but-valid-marker", secret, self.safety.key_id)
        protected = corrupted.encode("ascii")
        crc = zlib.crc32(protected) & 0xFFFFFFFF
        header = VitalPacketHeader(
            source_id=self.header.source_id,
            destination_id=self.header.destination_id,
            session_id=self.header.session_id,
            message_type=self.header.message_type,
            sequence_number=self.header.sequence_number,
            timestamp_ms=self.header.timestamp_ms,
            ttl_ms=self.header.ttl_ms,
            payload_length=len(protected),
        )
        mac = self._hmac(header, self.safety.packet_uuid, self.safety.key_id, protected, secret)
        return VitalSafePacket(
            header=header,
            safety=VitalPacketSafety(
                packet_uuid=self.safety.packet_uuid,
                crc32=crc,
                hmac_sha256=mac,
                key_id=self.safety.key_id,
            ),
            payload=dict(self.payload),
            encryption_enabled=True,
            encryption_algorithm=self.encryption_algorithm,
            encrypted_payload=corrupted,
            payload_format=self.payload_format,
        )

    def with_sequence_number(self, sequence_number: int, secret: str, new_uuid: bool = True) -> "VitalSafePacket":
        protected = self.encrypted_payload.encode("ascii") if self.encryption_enabled else canonical_payload(self.payload)
        header = VitalPacketHeader(
            source_id=self.header.source_id,
            destination_id=self.header.destination_id,
            session_id=self.header.session_id,
            message_type=self.header.message_type,
            sequence_number=int(sequence_number),
            timestamp_ms=self.header.timestamp_ms,
            ttl_ms=self.header.ttl_ms,
            payload_length=len(protected),
        )
        packet_uuid = str(uuid.uuid4()) if new_uuid else self.safety.packet_uuid
        mac = self._hmac(header, packet_uuid, self.safety.key_id, protected, secret)
        return VitalSafePacket(
            header=header,
            safety=VitalPacketSafety(
                packet_uuid=packet_uuid,
                crc32=zlib.crc32(protected) & 0xFFFFFFFF,
                hmac_sha256=mac,
                key_id=self.safety.key_id,
            ),
            payload=dict(self.payload),
            encryption_enabled=self.encryption_enabled,
            encryption_algorithm=self.encryption_algorithm,
            encrypted_payload=self.encrypted_payload,
            payload_format=self.payload_format,
        )


@dataclass
class VitalPacketValidationResult:
    accepted: bool
    result: str
    action: str
    reason: str


@dataclass
class VitalSession:
    local_id: str
    remote_id: str
    session_id: str
    key_id: str = "SIM_KEY_01"
    secret: str = "cbtc-sim-shared-secret"
    state: RastaSessionState = RastaSessionState.ESTABLISHED
    last_sequence_number: int = -1
    accepted_uuids: Set[str] = field(default_factory=set)

    def validate(self, packet: VitalSafePacket, now_ms: int) -> VitalPacketValidationResult:
        if self.state == RastaSessionState.LOST:
            return VitalPacketValidationResult(False, "TIMEOUT", "ignored", "watchdog timeout")
        if packet.header.destination_id != self.local_id:
            return VitalPacketValidationResult(False, "REJECTED", "ignored", "wrong destination")
        if packet.header.source_id != self.remote_id:
            return VitalPacketValidationResult(False, "REJECTED", "ignored", "wrong source")
        if packet.header.session_id != self.session_id:
            return VitalPacketValidationResult(False, "REJECTED", "ignored", "wrong session")
        if now_ms - packet.header.timestamp_ms > packet.header.ttl_ms:
            return VitalPacketValidationResult(False, "TIMEOUT", "ignored", "expired TTL")
        protected_payload_bytes = packet.encrypted_payload.encode("ascii") if packet.encryption_enabled else canonical_payload(packet.payload)
        if len(protected_payload_bytes) != packet.header.payload_length:
            return VitalPacketValidationResult(False, "REJECTED", "ignored", "payload length mismatch")
        crc = zlib.crc32(protected_payload_bytes) & 0xFFFFFFFF
        if crc != packet.safety.crc32:
            return VitalPacketValidationResult(False, "CRC_ERROR", "ignored", "CRC error")
        expected_hmac = VitalSafePacket._hmac(
            packet.header,
            packet.safety.packet_uuid,
            packet.safety.key_id,
            protected_payload_bytes,
            self.secret,
        )
        if not hmac.compare_digest(expected_hmac, packet.safety.hmac_sha256):
            return VitalPacketValidationResult(False, "HMAC_ERROR", "ignored", "HMAC error")
        if packet.encryption_enabled:
            try:
                payload_bytes = decrypt_payload_bytes(packet.encrypted_payload, self.secret, packet.safety.key_id)
                json.loads(payload_bytes.decode("utf-8"))
            except Exception as exc:
                return VitalPacketValidationResult(False, "DECRYPT_ERROR", "ignored", str(exc))
        if packet.safety.packet_uuid in self.accepted_uuids or packet.header.sequence_number == self.last_sequence_number:
            return VitalPacketValidationResult(False, "REPLAY", "ignored", "duplicate/replay packet")
        if packet.header.sequence_number < self.last_sequence_number:
            return VitalPacketValidationResult(False, "OUT_OF_ORDER", "ignored", "out-of-order sequence")
        self.accepted_uuids.add(packet.safety.packet_uuid)
        self.last_sequence_number = packet.header.sequence_number
        self.state = RastaSessionState.ESTABLISHED
        return VitalPacketValidationResult(True, "ACCEPTED", "state updated", "accepted")
