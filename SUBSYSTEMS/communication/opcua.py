from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict

from .rasta import canonical_payload, decrypt_payload_bytes, encrypt_payload_bytes


@dataclass(frozen=True)
class OpcUaSupervisionFrame:
    request_id: str
    response_id: str
    source_id: str
    destination_id: str
    method_name: str
    timestamp_ms: int
    timeout_ms: int
    retry_count: int
    encrypted_flag: bool
    certificate_id: str
    payload: Dict[str, Any] = field(default_factory=dict)
    encryption_enabled: bool = True
    encryption_algorithm: str = "SIM_AES"
    key_id: str = "SIM_KEY_01"
    encrypted_payload: str = ""
    payload_format: str = "json"

    def __post_init__(self):
        if self.encryption_enabled and not self.encrypted_payload:
            encrypted = encrypt_payload_bytes(canonical_payload(self.payload), "cbtc-sim-shared-secret", self.key_id)
            object.__setattr__(self, "encrypted_payload", encrypted)
        if not self.encryption_enabled:
            object.__setattr__(self, "encryption_algorithm", "NONE")

    def is_vital_forbidden(self) -> bool:
        return self.method_name.upper() in {"MA_UPDATE", "EOA", "SPEED_PROFILE", "MOVEMENT_AUTHORITY"}

    def decoded_payload(self, secret: str = "cbtc-sim-shared-secret") -> Dict[str, Any]:
        if not self.encryption_enabled:
            return dict(self.payload)
        import json

        payload_bytes = decrypt_payload_bytes(self.encrypted_payload, secret, self.key_id)
        decoded = json.loads(payload_bytes.decode("utf-8"))
        if not isinstance(decoded, dict):
            raise ValueError("decoded OPC UA-like payload is not an object")
        return decoded
