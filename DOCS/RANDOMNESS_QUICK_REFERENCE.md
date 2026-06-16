# Quick Reference: Stochastic Elements Summary

## 📊 Visual Overview

```
┌─────────────────────────────────────────────────────────────────┐
│         RANDOMNESS IN CBTC_SIM SIMULATION                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ 1. NETWORK COMMUNICATION LAYER                                 │
│    ├─ Jitter:     0-18ms (RED) / 0-22ms (BLUE)               │
│    │              [Uniform Distribution]                       │
│    ├─ Latency:    70-88ms (RED) / 80-102ms (BLUE)            │
│    │              [Base + Jitter + Multiplier]                 │
│    └─ Packet Loss: 0-100% (depends on state + fault)         │
│                   [Bernoulli Trial]                            │
│                                                                 │
│ 2. DCS TRANSPORT LAYER                                         │
│    └─ DCS Delay:  50-350 ms  [Uniform Distribution]           │
│                                                                 │
│ 3. TRAIN POSITIONING LAYER                                     │
│    └─ Odometer Error: ±5% × distance  [Deterministic]        │
│                                                                 │
│ 4. SIGNAL QUALITY LAYER                                        │
│    └─ BER:         0-20%  [Bernoulli + Modulation Model]     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 🎯 Configuration Parameters

### Fixed Parameters (CONFIG/config.py)
```yaml
ODOMETER_ERROR_RATE: 0.05          # ±5% of distance
DCS_DELAY_MIN_S: 0.05              # 50 ms
DCS_DELAY_MAX_S: 0.35              # 350 ms
```

### Dynamic Parameters (Changeable during simulation)
```yaml
RED Path:
  base_latency_ms: 70
  jitter_ms: 18
  packet_loss_rate: 0.0 (adjustable)

BLUE Path:
  base_latency_ms: 80
  jitter_ms: 22
  packet_loss_rate: 0.0 (adjustable)

Faults (toggleable):
  - radio_coverage_loss
  - packet_loss (+45%)
  - high_latency (×4.0)
  - crc_corruption
  - hmac_corruption
  - ber_corruption
```

## 📈 Packet Loss Rate Calculation

```
loss_rate = base_loss + edge_loss + state_loss + fault_loss

Where:
├─ base_loss:  path.packet_loss_rate      (default 0%)
├─ edge_loss:  edge_factor × 0.25         (0-25%)
├─ state_loss: DEGRADED → +15%
│              LOST → 100%
└─ fault_loss: packet_loss fault → +45%
```

### Example Scenarios:

**Scenario A: Normal, center coverage**
```
loss_rate = 0% + 0.0×0.25 + 0% + 0% = 0%
Packets:   0% lost (all pass)
```

**Scenario B: DEGRADED, edge coverage**
```
loss_rate = 0% + 1.0×0.25 + 15% + 0% = 40%
Packets:   40% lost (random)
```

**Scenario C: DEGRADED + fault enabled, edge coverage**
```
loss_rate = 0% + 1.0×0.25 + 15% + 45% = 85%
Packets:   85% lost (most fail)
```

**Scenario D: LOST state**
```
loss_rate = 100%
Packets:   100% lost (all fail)
```

## 🔄 How Randomness Works

### 1. Network Jitter
```python
import random
jitter_ms = random.uniform(0.0, 18.0)  # RED path
# Result: 0-18 ms (any value equally likely)
```

### 2. DCS Communication Delay
```python
delay_s = random.uniform(0.05, 0.35)   # 50-350 ms
# Applied to each packet sent to ZC
```

### 3. Packet Loss Decision
```python
loss_rate = 0.25  # 25% loss rate
if random.random() < 0.25:  # 25% chance
    # Packet is lost
    path.lost_count += 1
```

### 4. Odometer Error
```python
ODOMETER_ERROR_RATE = 0.05
distance_since_fix = 100  # meters
error = 0.05 × 100 = 5 meters
pos_error = ±5m  # Sign alternates
```

## 📊 Probability Distributions Used

| Element | Type | Formula | Visual |
|---------|------|---------|--------|
| **Jitter** | Uniform | `U[0, jitter_max]` | ▁▁▁▁▁ |
| **DCS Delay** | Uniform | `U[0.05, 0.35]` | ▁▁▁▁▁ |
| **Packet Loss** | Bernoulli | `p < loss_rate` | ▏▁▁▁▏ |
| **BER Error** | Bernoulli | `p < ber_sim` | ▏▁▁▁▏ |
| **Odometer** | Deterministic | `5% × dist` | Fixed |

## 🎮 Simulation Control Examples

### Get current path metrics
```python
red_path = sim.dcs_transport.paths["RED"]
print(f"RED Latency: {red_path.base_latency_ms} ms")
print(f"RED Jitter: {red_path.jitter_ms} ms")
print(f"Loss rate: {red_path.loss_rate():.1%}")
```

### Enable packet loss fault
```python
sim.dcs_transport.set_fault("packet_loss", True)
# Adds 45% to loss_rate
```

### Degrade a path
```python
sim.dcs_transport.set_path_state("RED", "DEGRADED")
# Multiplies latency by 2x, adds 15% to loss_rate
```

### Adjust loss rate
```python
sim.dcs_transport.paths["RED"].packet_loss_rate = 0.10  # 10%
```

## 🔍 Monitoring Randomness Effects

### In Engineering Panel
```
Statistics Tab shows:
├─ Packet Loss: X.X%
├─ Latency: min-max ms
├─ Jitter: current jitter
└─ Timeout Count: N packets
```

### In Train Panel
```
Reported Position:
├─ Actual: 350.5 m
├─ Reported: 354.2 m (with error)
└─ Error: +3.7 m (from odometer)
```

## ✅ Verification Checklist

- [ ] Jitter is applied to each packet (0-18/22 ms)
- [ ] DCS delay is random (50-350 ms per send)
- [ ] Packet loss follows loss_rate probability
- [ ] Odometer error accumulates (±5% per distance)
- [ ] Edge coverage affects latency/loss (0-25% extra)
- [ ] DEGRADED state increases delay (×2) and loss (+15%)
- [ ] Faults can be toggled to increase randomness
- [ ] All randomness is deterministic based on params

## 📝 Notes

1. **Randomness is NOT arbitrary** - All values follow defined ranges
2. **Reproducible** - With same seed, simulation produces same results
3. **Configurable** - Parameters can be adjusted before/during run
4. **Observable** - All random effects are logged and displayed
5. **Physical** - Models real network behavior (jitter, loss, delay)

---

Generated from CBTC_SIM analysis
See `RANDOMNESS_AND_STOCHASTIC_ELEMENTS.md` for detailed documentation
