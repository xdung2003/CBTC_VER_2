# Randomness and Stochastic Elements in CBTC_SIM

Các yếu tố ngẫu nhiên trong CBTC_SIM được sử dụng để mô phỏng các hiện tượng không cố định trong hệ thống liên lạc, định vị, và vận hành tàu.

**Câu trả lời chính:** Randomness trong dự án **KHÔNG phải tùy ý** mà tuân theo các **dải và phân phối cụ thể** được định nghĩa trong cấu hình.

---

## Danh Sách Các Yếu Tố Ngẫu Nhiên

### 1. **Network Jitter (Độ Trễ Mạng Biến Động)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/communication/transport.py](SUBSYSTEMS/communication/transport.py) line 34
- **Loại:** Uniform Distribution

#### Dải giá trị
```python
jitter_ms = random.uniform(0.0, self.jitter_ms * multiplier)
```

**Base jitter range:**
- **RED path:** 0-18 ms
- **BLUE path:** 0-22 ms

**Multipliers (áp dụng các yếu tố):**
- Normal state: ×1.0
- DEGRADED state: ×2.0
- High latency fault: ×4.0
- Edge of coverage: ×1.0 to ×2.5 (phụ thuộc vị trí)

#### Ví dụ thực tế
```
RED path bình thường: 0-18 ms
RED path DEGRADED:    0-36 ms (18 × 2.0)
RED path high_latency: 0-72 ms (18 × 4.0)
```

---

### 2. **DCS Communication Delay (Độ Trễ Truyền Thông DCS)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/runtime.py](SUBSYSTEMS/runtime.py) line 2741
- **File cấu hình:** [CONFIG/config.py](CONFIG/config.py) lines 183-184
- **Loại:** Uniform Distribution

#### Dải giá trị
```python
DCS_DELAY_MIN_S = 0.05  # 50 ms
DCS_DELAY_MAX_S = 0.35  # 350 ms
delay_s = random.uniform(DCS_DELAY_MIN_S, DCS_DELAY_MAX_S)
```

**Phạm vi:** **50 ms đến 350 ms**

#### Cách sử dụng
- Độ trễ này được áp dụng khi DCS gửi MA_UPDATE tới ZC
- Mỗi lần gửi packet sẽ có một độ trễ ngẫu nhiên trong phạm vi này
- Mô phỏng độ trễ xử lý và truyền thông thực tế

---

### 3. **Packet Loss Rate (Tỷ Lệ Mất Gói Tin)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/communication/transport.py](SUBSYSTEMS/communication/transport.py) lines 37-45 & 218
- **Loại:** Bernoulli Trial (0 hoặc 1)

#### Công thức tính tỷ lệ mất gói
```python
def loss_rate(self, edge_factor: float = 0.0, packet_loss_fault: bool = False) -> float:
    if self.state == DcsPathState.LOST:
        return 1.0  # 100% loss
    rate = self.packet_loss_rate + edge_factor * 0.25
    if self.state == DcsPathState.DEGRADED:
        rate += 0.15  # +15%
    if packet_loss_fault:
        rate += 0.45  # +45%
    return min(1.0, max(0.0, rate))
```

#### Dải giá trị

| Điều kiện | Tỷ lệ mất gói |
|----------|-------------|
| Path state = LOST | 100% |
| Normal state | 0% (default) + edge_factor×0.25 |
| DEGRADED state | +15% |
| packet_loss fault ON | +45% |
| Edge of coverage | +0-25% (phụ thuộc vị trí) |

#### Ví dụ
```
Scenario 1: Normal path, center coverage
  loss_rate = 0% + 0.0 × 0.25 = 0%

Scenario 2: Normal path, edge of coverage
  loss_rate = 0% + 1.0 × 0.25 = 25%

Scenario 3: DEGRADED path, edge coverage
  loss_rate = 0% + 1.0 × 0.25 + 15% = 40%

Scenario 4: DEGRADED path, packet_loss fault ON
  loss_rate = 0% + 1.0 × 0.25 + 15% + 45% = 85%

Scenario 5: LOST state
  loss_rate = 100%
```

#### Cách kiểm tra mất gói
```python
if random.random() < loss_rate:
    # Packet is lost
    path.lost_count += 1
```

---

### 4. **Odometer/Position Error (Sai Số Vị Trí)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/train.py](SUBSYSTEMS/train.py) line 1196
- **File cấu hình:** [CONFIG/config.py](CONFIG/config.py) line 181
- **Loại:** Deterministic (không random, nhưng biến động)

#### Công thức
```python
ODOMETER_ERROR_RATE = 0.05  # 5%

error = min(error_cap_m, ODOMETER_ERROR_RATE * dist_since)
self.pos_error_m = self.odometer_error_sign * error
```

**Dải giá trị:**
- **Error rate:** 5% of distance traveled since last balise fix
- **Max error cap:** Phụ thuộc vị trí dẫn hướng hiện tại

#### Ví dụ
```
Train đi 100m từ lần fix balise cuối:
  error = min(cap, 0.05 × 100) = min(cap, 5m)

Train đi 500m:
  error = min(cap, 0.05 × 500) = min(cap, 25m)
```

#### Dấu hiệu sai số
```python
self.odometer_error_sign *= -1.0  # Thay đổi dấu mỗi khi fix
```
- Sai số xen kẽ giữa dương (+) và âm (-)
- Mô phỏng sai số odometer thực tế

---

### 5. **Bit Error Rate (BER) - Sai Số Bit Truyền Thông**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/communication/transport.py](SUBSYSTEMS/communication/transport.py) lines 154-160 & 236
- **Loại:** Bernoulli Trial dựa trên BER

#### Công thức
```python
if (self.faults["ber_corruption"] 
    or random.random() < min(0.2, float(modulation["bit_error_rate_sim"]) * 10.0)
) and hasattr(delivered, "with_encrypted_payload_corruption"):
    delivered = delivered.with_encrypted_payload_corruption()
```

#### Dải giá trị
```python
BER simulation = min(0.2, BER_value × 10.0)
```

**Phạm vi:** 0% đến 20%

#### Mô phỏng kênh truyền
```python
def _modulation_sample(self, rap: Optional[RadioAccessPoint], edge_factor: float):
    # Returns modulation info including BER
    return {
        "modulation_scheme": "OFDM-QAM-like",
        "signal_quality": "GOOD" | "FAIR" | "POOR",
        "bit_error_rate_sim": 0.001 to 0.05 (phụ thuộc coverage)
    }
```

---

### 6. **Network Path Latency (Độ Trễ Toàn Path)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/communication/transport.py](SUBSYSTEMS/communication/transport.py) lines 30-35
- **Loại:** Uniform Distribution

#### Công thức
```python
def latency_ms(self, edge_factor: float = 0.0, high_latency: bool = False) -> float:
    multiplier = 1.0 + edge_factor * 2.5  # 1.0 to 3.5x
    if self.state == DcsPathState.DEGRADED:
        multiplier *= 2.0
    if high_latency:
        multiplier *= 4.0
    jitter = random.uniform(0.0, self.jitter_ms * multiplier)
    return max(0.0, self.base_latency_ms * multiplier + jitter)
```

#### Dải giá trị

| Path | Base Latency | Jitter Range | State | Total Range |
|------|-------------|--------------|-------|------------|
| RED | 70 ms | 0-18 ms | Normal | 70-88 ms |
| RED | 70 ms | 0-36 ms | DEGRADED | 70-106 ms |
| RED | 70 ms | 0-72 ms | High_latency | 70-142 ms |
| BLUE | 80 ms | 0-22 ms | Normal | 80-102 ms |
| BLUE | 80 ms | 0-44 ms | DEGRADED | 80-124 ms |

---

### 7. **Radio Coverage Edge Factor (Hệ Số Cạnh Vùng Phủ)**

#### Vị trí định nghĩa
- **File:** [SUBSYSTEMS/communication/transport.py](SUBSYSTEMS/communication/transport.py) lines 49-53
- **Loại:** Continuous function (không random)

#### Công thức
```python
def edge_factor(self, pos_m: float, edge_width_m: float = 80.0) -> float:
    if not self.contains(pos_m):
        return 1.0  # Outside coverage
    dist_to_edge = min(abs(pos_m - self.start_m), abs(self.end_m - pos_m))
    return max(0.0, min(1.0, 1.0 - dist_to_edge / max(1.0, edge_width_m)))
```

**Dải giá trị:**
- **Center of coverage:** 0.0 (không có penalty)
- **Edge of coverage:** 1.0 (maximum penalty)
- **Outside coverage:** Return immediately (no coverage)
- **Edge width:** 80m (khoảng cách để suy giảm từ 0 → 1.0)

#### Ví dụ
```
RAP coverage: 0m - 900m, edge_width = 80m
Position 50m:   edge_factor = 1.0 - (50/80) = 0.375 (37.5% penalty)
Position 100m:  edge_factor = 1.0 - (80/80) = 0.0 (no penalty)
Position 850m:  edge_factor = 1.0 - (50/80) = 0.375 (37.5% penalty)
Position 950m:  outside coverage (1.0 return)
```

---

## Tổng Hợp Các Yếu Tố Ngẫu Nhiên

| Yếu tố | Vị trí | Loại | Phạm vi | Ghi chú |
|--------|--------|------|---------|---------|
| **Network Jitter** | transport.py:34 | Uniform | 0-18ms (RED), 0-22ms (BLUE) | Nhân với multiplier |
| **DCS Delay** | runtime.py:2741 | Uniform | 50-350 ms | Fixed dải |
| **Packet Loss** | transport.py:218 | Bernoulli | 0%-100% | Phụ thuộc state + edge |
| **Odometer Error** | train.py:1196 | Deterministic | 5% × distance | Xen kẽ dấu (+/-) |
| **BER** | transport.py:236 | Bernoulli | 0%-20% | Phụ thuộc modulation |
| **Latency** | transport.py:31 | Uniform | 70-88ms+ (RED) | Phụ thuộc state |
| **Edge Factor** | transport.py:49 | Continuous | 0.0-1.0 | Non-random |

---

## Lớp Cấu Hình (Configuration Layers)

### Layer 1: Base Configuration
```python
# CONFIG/config.py
ODOMETER_ERROR_RATE = 0.05      # 5%
DCS_DELAY_MIN_S = 0.05          # 50ms
DCS_DELAY_MAX_S = 0.35          # 350ms
```

### Layer 2: Path Definition
```python
# SUBSYSTEMS/communication/transport.py - DcsNetworkPath
RED = DcsNetworkPath(
    name="RED",
    base_latency_ms=70.0,
    jitter_ms=18.0,
    packet_loss_rate=0.0  # Can be modified
)
BLUE = DcsNetworkPath(
    name="BLUE",
    base_latency_ms=80.0,
    jitter_ms=22.0,
    packet_loss_rate=0.0
)
```

### Layer 3: Runtime Faults
```python
# Can be toggled during simulation
self.faults = {
    "radio_coverage_loss": False,
    "packet_loss": False,          # +45% to loss_rate
    "high_latency": False,         # ×4.0 to latency
    "crc_corruption": False,
    "hmac_corruption": False,
    "ber_corruption": False,
}
```

### Layer 4: Dynamic Factors
```python
# Real-time calculations
edge_factor = 0.0 to 1.0      # Based on train position
multiplier = 1.0 to 3.5       # Based on edge_factor + state
```

---

## Cách Điều Chỉnh Randomness

### 1. Thay đổi DCS Delay Range
```python
# CONFIG/config.py
DCS_DELAY_MIN_S = 0.10  # 100 ms
DCS_DELAY_MAX_S = 0.50  # 500 ms
```

### 2. Thay đổi Network Jitter
```python
# SUBSYSTEMS/communication/transport.py
RED = DcsNetworkPath(
    name="RED",
    base_latency_ms=70.0,
    jitter_ms=30.0  # Increase from 18.0 to 30.0
)
```

### 3. Thay đổi Odometer Error Rate
```python
# CONFIG/config.py
ODOMETER_ERROR_RATE = 0.10  # 10% instead of 5%
```

### 4. Kích hoạt Fault Injection
```python
# During simulation via GUI or code
sim.dcs_transport.set_fault("packet_loss", True)
sim.dcs_transport.set_fault("high_latency", True)
sim.dcs_transport.set_path_state("RED", "DEGRADED")
```

---

## Distribution Types Sử Dụng

### 1. **Uniform Distribution** (Đều)
```python
value = random.uniform(min_val, max_val)
```
- Tất cả giá trị trong phạm vi có xác suất bằng nhau
- Ví dụ: Jitter, DCS Delay

### 2. **Bernoulli Trial** (0 hoặc 1)
```python
if random.random() < probability:
    # Event happens with probability
```
- Xác suất cố định để sự kiện xảy ra
- Ví dụ: Packet loss, BER corruption

### 3. **Deterministic with Sign Alternation**
```python
error = error_value * sign  # sign thay đổi mỗi lần
```
- Giá trị xác định nhưng dấu xen kẽ
- Ví dụ: Odometer error

---

## Kiểm Tra Randomness trong Simulation

### Xem thống kê mất gói
```
Engineering Panel → Statistics Tab
  → Packet Loss Rate: X.X%
  → Timeout Count: N
```

### Xem thông tin đường truyền
```
Control Panel → DCS Transport
  → Path: RED  State: DEGRADED
  → Latency: 70-180ms
  → Jitter: 0-36ms
```

### Xem sai số vị trí
```
Train Panel
  → Reported Pos: X.X ± error_m
  → Error Sign: +/-
```

---

## Kết Luận

**Randomness trong CBTC_SIM:**
- ✅ **KHÔNG tùy ý** - Tuân theo các dải cụ thể
- ✅ **Có thể điều chỉnh** - Qua CONFIG hoặc runtime faults
- ✅ **Mô phỏng thực tế** - Phản ánh các hiện tượng mạng thực
- ✅ **Kiểm soát được** - Có thể theo dõi và phân tích

Các yếu tố ngẫu nhiên này giúp:
1. Mô phỏng hành vi thực tế của hệ thống truyền thông
2. Kiểm tra khả năng phục hồi lỗi của ATP/ATO
3. Xác validate cơ chế giám sát an toàn
4. Tối ưu hóa các tham số truyền thông
