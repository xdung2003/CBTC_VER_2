# Implementation Details: Where Randomness is Used in Code

## 1. Network Jitter Implementation

### Location: `SUBSYSTEMS/communication/transport.py` (Line 30-35)

```python
@dataclass
class DcsNetworkPath:
    name: str
    state: DcsPathState = DcsPathState.OK
    base_latency_ms: float = 70.0
    jitter_ms: float = 20.0
    packet_loss_rate: float = 0.0

    def latency_ms(self, edge_factor: float = 0.0, high_latency: bool = False) -> float:
        # Apply multipliers based on state
        multiplier = 1.0 + edge_factor * 2.5
        if self.state == DcsPathState.DEGRADED:
            multiplier *= 2.0
        if high_latency:
            multiplier *= 4.0
        
        # RANDOMNESS: Generate random jitter
        jitter = random.uniform(0.0, self.jitter_ms * multiplier)
        
        return max(0.0, self.base_latency_ms * multiplier + jitter)
```

### Usage Example
```python
# RED path in normal state
path = DcsNetworkPath("RED", base_latency_ms=70.0, jitter_ms=18.0)
latency = path.latency_ms(edge_factor=0.5)
# Result: 70 * 1.25 + random.uniform(0, 18*1.25) = 87.5 + 0-22.5ms
```

### Configuration
```python
# RED path: base 70ms, jitter 0-18ms
RED = DcsNetworkPath("RED", base_latency_ms=70.0, jitter_ms=18.0)

# BLUE path: base 80ms, jitter 0-22ms
BLUE = DcsNetworkPath("BLUE", base_latency_ms=80.0, jitter_ms=22.0)
```

---

## 2. DCS Communication Delay

### Location: `SUBSYSTEMS/runtime.py` (Line 2741)

```python
def _collect_zc_status_packet(self, with_delay: bool = True) -> Optional[VitalPacket]:
    """Collect ZC status with optional random delay."""
    
    # Apply random delay to each packet
    delay_s = random.uniform(DCS_DELAY_MIN_S, DCS_DELAY_MAX_S) if with_delay else 0.0
    
    # Build the packet...
    packet = build_zc_status_packet(...)
    
    # Schedule delivery with delay
    self.pending_zc_position_packets.append((self.sim_time_s + delay_s, packet))
```

### Configuration in `CONFIG/config.py`
```python
DCS_DELAY_MIN_S = 0.05      # 50 milliseconds
DCS_DELAY_MAX_S = 0.35      # 350 milliseconds
```

### What This Means
- Every ZC packet sent has a random delay between 50-350ms
- This delay is uniformly distributed (all values equally likely)
- Different packets can have different delays

### Example Timeline
```
t=0.0s:  Packet A ready → delay = 0.087s → delivered at 0.087s
t=0.2s:  Packet B ready → delay = 0.223s → delivered at 0.423s
t=0.4s:  Packet C ready → delay = 0.051s → delivered at 0.451s
```

---

## 3. Packet Loss Rate Calculation

### Location: `SUBSYSTEMS/communication/transport.py` (Line 37-45)

```python
def loss_rate(self, edge_factor: float = 0.0, packet_loss_fault: bool = False) -> float:
    """Calculate packet loss probability based on state and faults."""
    
    # Handle LOST state specially
    if self.state == DcsPathState.LOST:
        return 1.0  # 100% loss
    
    # Start with base loss rate (default 0%)
    rate = self.packet_loss_rate + edge_factor * 0.25
    
    # Add loss for degraded path
    if self.state == DcsPathState.DEGRADED:
        rate += 0.15  # +15%
    
    # Add loss for fault condition
    if packet_loss_fault:
        rate += 0.45  # +45%
    
    # Clamp to [0, 1]
    return min(1.0, max(0.0, rate))
```

### How It's Used

```python
# Location: transport.py line 218
loss_rate = path.loss_rate(edge_factor, self.faults["packet_loss"])

# RANDOMNESS: Decide if packet is lost
if random.random() < loss_rate:
    path.lost_count += 1
    path.timeout_count += 1
    return None, now_s, timeout_event
```

### Examples

**Example 1: Normal State, Center**
```python
path = RED  # Normal state
edge_factor = 0.0  # Center of coverage
loss_rate = 0.0 + 0.0*0.25 = 0%

random.random() = 0.5
0.5 < 0.0? → No → Packet passes
```

**Example 2: Degraded State, Edge**
```python
path = RED  # DEGRADED state
edge_factor = 1.0  # Edge of coverage
fault = False

loss_rate = 0.0 + 1.0*0.25 + 0.15 + 0.0 = 0.40 (40%)

random.random() = 0.35
0.35 < 0.40? → Yes → Packet lost
```

**Example 3: Fault Enabled, Degraded, Edge**
```python
path = RED
state = DEGRADED
edge_factor = 1.0
fault = True

loss_rate = 0.0 + 1.0*0.25 + 0.15 + 0.45 = 0.85 (85%)

random.random() = 0.70
0.70 < 0.85? → Yes → Packet lost (high probability)
```

---

## 4. Odometer/Position Error

### Location: `SUBSYSTEMS/train.py` (Line 1185-1210)

```python
def update_position_error(self, now_s: float):
    """Update position error based on odometer accuracy."""
    
    # When balise fix happens
    if self.beacon_position_locked:
        self.sync_reported_position()
        return
    
    # Calculate error accumulation since last balise
    dist_since = max(0.0, self.pos - self.last_balise_pos)
    
    # RANDOMNESS: Error grows proportional to distance
    error = min(error_cap_m, ODOMETER_ERROR_RATE * dist_since)
    
    # IMPORTANT: Error sign alternates
    self.pos_error_m = self.odometer_error_sign * error
    self.reported_pos = self.pos + self.pos_error_m
```

### Configuration

```python
# CONFIG/config.py
ODOMETER_ERROR_RATE = 0.05  # 5% of distance traveled
```

### How Error Sign Changes

```python
# When balise fix occurs
if self.beacon_position_locked:
    self.odometer_error_sign *= -1.0  # Flip sign
    self.pos_error_m = 0.0  # Reset to zero
```

### Timeline Example

```
t=0s:   Train at balise, pos=0.0, error=0.0, sign=+1
t=10s:  Train at 100m, error = min(cap, 0.05*100) = 5m
        reported_pos = 100 + 5 = 105m (5m ahead)

t=20s:  Balise fix! sign flips: +1 → -1, error reset to 0

t=30s:  Train at 200m, error = min(cap, 0.05*100) = 5m  
        reported_pos = 200 - 5 = 195m (5m behind)
```

---

## 5. Bit Error Rate (BER) Corruption

### Location: `SUBSYSTEMS/communication/transport.py` (Line 154-160 & 236)

```python
def _modulation_sample(self, rap: Optional[RadioAccessPoint], edge_factor: float):
    """Get modulation characteristics for current position."""
    
    if not self.radio_physical.get("enabled", True):
        return {"bit_error_rate_sim": 0.0, ...}
    
    # Calculate signal quality based on coverage
    if rap is None:
        signal_quality = "POOR"
        ber_base = 0.05
    elif edge_factor > 0.7:
        signal_quality = "FAIR"
        ber_base = 0.02
    else:
        signal_quality = "GOOD"
        ber_base = 0.001
    
    return {
        "modulation_scheme": "OFDM-QAM-like",
        "signal_quality": signal_quality,
        "bit_error_rate_sim": ber_base,
    }
```

### Usage

```python
# Location: transport.py line 236
modulation = self._modulation_sample(rap, edge_factor)

# RANDOMNESS: Decide if packet gets BER corruption
if (
    self.faults["ber_corruption"]
    or random.random() < min(0.2, float(modulation["bit_error_rate_sim"]) * 10.0)
) and hasattr(delivered, "with_encrypted_payload_corruption"):
    delivered = delivered.with_encrypted_payload_corruption()
```

### Examples

**Signal Quality Impact:**
```
Coverage Quality | BER Base | Random Check | Corruption Rate
GOOD (center)    | 0.001    | < 0.01       | ~1%
FAIR (edge)      | 0.02     | < 0.2        | ~20%
POOR (outside)   | 0.05     | < 0.2        | ~20% (capped)
```

---

## 6. Edge Coverage Factor

### Location: `SUBSYSTEMS/communication/transport.py` (Line 49-53)

```python
@dataclass(frozen=True)
class RadioAccessPoint:
    id: str
    start_m: float
    end_m: float

    def contains(self, pos_m: float) -> bool:
        return self.start_m <= pos_m <= self.end_m

    def edge_factor(self, pos_m: float, edge_width_m: float = 80.0) -> float:
        """Calculate penalty for being at edge of coverage (0.0 to 1.0)."""
        if not self.contains(pos_m):
            return 1.0  # Outside coverage = maximum penalty
        
        # Distance to nearest edge
        dist_to_edge = min(
            abs(pos_m - self.start_m),  # Distance to start
            abs(self.end_m - pos_m)      # Distance to end
        )
        
        # Penalty decreases linearly over 80m
        return max(0.0, min(1.0, 1.0 - dist_to_edge / max(1.0, edge_width_m)))
```

### Usage Impact

```python
# How edge_factor affects loss rate
loss_rate = packet_loss_rate + edge_factor * 0.25

# How edge_factor affects latency
multiplier = 1.0 + edge_factor * 2.5
latency = base_latency * multiplier + jitter
```

### Coverage Example

```
RAP coverage: 0m ────────────── 900m
              |←─── 80m ───→|←─── 80m ───→|

Position  50m: edge_factor = 1.0 - 50/80  = 0.375 (37.5% penalty)
Position 100m: edge_factor = 1.0 - 80/80  = 0.0   (no penalty)
Position 500m: edge_factor = 0.0          (center)
Position 820m: edge_factor = 1.0 - 80/80  = 0.0
Position 850m: edge_factor = 1.0 - 50/80  = 0.375
Position 950m: edge_factor = 1.0          (outside - max penalty)
```

---

## 7. Radio Coverage Management

### Location: `SUBSYSTEMS/communication/transport.py` (Line 167-173)

```python
def coverage_for_position(self, pos_m: float) -> Tuple[Optional[RadioAccessPoint], float, bool]:
    """Get radio coverage info for train position."""
    
    # Find all RAPs covering this position
    covering = [rap for rap in self.radio_access_points if rap.contains(pos_m)]
    
    # Check for coverage loss fault
    if not covering or self.faults["radio_coverage_loss"]:
        return None, 1.0, False  # No coverage
    
    # Use closest RAP to minimize edge_factor penalty
    rap = min(covering, key=lambda item: item.edge_factor(pos_m))
    
    # Mark if in overlap zone (multiple RAPs)
    overlap = len(covering) > 1
    
    return rap, rap.edge_factor(pos_m), overlap
```

---

## Summary Table

| Element | File | Line | Randomness Type | Range |
|---------|------|------|-----------------|-------|
| **Jitter** | transport.py | 34 | `random.uniform()` | 0-18ms (RED) |
| **Latency** | transport.py | 31 | Base + Jitter | 70-88ms (RED) |
| **Loss Rate** | transport.py | 37-45 | Formula-based | 0-100% |
| **Packet Loss** | transport.py | 218 | `random.random()` | Bernoulli(p) |
| **DCS Delay** | runtime.py | 2741 | `random.uniform()` | 50-350ms |
| **Odometer Error** | train.py | 1196 | Deterministic | ±5% × dist |
| **BER Check** | transport.py | 236 | `random.random()` | Bernoulli(p) |
| **Edge Factor** | transport.py | 49 | Deterministic | 0-1.0 |

---

## Key Takeaways

1. **All randomness is intentional** - Uses specific distributions
2. **Ranges are defined** - Not arbitrary values
3. **Reproducible** - Same config → same behavior (with fixed seed)
4. **Controllable** - Can be modified via CONFIG or runtime faults
5. **Observable** - All effects are logged and measurable

Để thay đổi bất kỳ yếu tố ngẫu nhiên nào, sửa các giá trị trong:
- `CONFIG/config.py` - Cấu hình cố định
- `SUBSYSTEMS/communication/transport.py` - Đường truyền mạng
- Runtime faults - Điều khiển khi chạy
