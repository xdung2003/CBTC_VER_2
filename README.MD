# CBTC_SIM - Hệ Thống Mô Phỏng Điều Khiển Tàu Dựa Trên Truyền Thông

Một nền tảng mô phỏng cho Hệ Thống Điều Khiển Tàu Dựa Trên Truyền Thông (CBTC - Communications-Based Train Control). Dự án cung cấp mô phỏng chi tiết về các thành phần chính: ATP (Automatic Train Protection), ATO (Automatic Train Operation), ATS (Automatic Train Supervision), ZC (Zone Controller) và DCS (Dispatch Control System).

## Mục Lục

- [Cài Đặt](#cài-đặt)
- [Hướng Dẫn Chạy Nhanh](#hướng-dẫn-chạy-nhanh)
- [Kiến Trúc Hệ Thống](#kiến-trúc-hệ-thống)
- [Các Tính Năng Chính](#các-tính-năng-chính)

## Cài Đặt

### Yêu Cầu Hệ Thống

- Python 3.8 trở lên
- Windows 10+, Linux (Ubuntu 18.04+), macOS 10.14+
- RAM tối thiểu: 2GB
- Dung lượng đĩa: 500MB

### Cài Đặt Dependencies

Sao chép kho lưu trữ:

```bash
git clone <repository-url>
cd CBTC_SIM
```

Cài đặt các thư viện cần thiết:

```bash
pip install -r requirements.txt
```

Hoặc cài đặt PyYAML thủ công:

```bash
pip install pyyaml
```

### Xác Minh Cài Đặt

```bash
# Kiểm tra Python
python --version

# Kiểm tra PyYAML
python -c "import yaml; print('PyYAML OK')"

# Chạy ứng dụng
python run.py
```

## Hướng Dẫn Chạy Nhanh

### Chạy Ứng Dụng

Từ thư mục gốc dự án, chạy lệnh:

```bash
python run.py
```

Ứng dụng sẽ khởi động với giao diện GUI. Kịch bản mặc định sẽ được tải từ DOCS/default_scenario.yaml.

### Giao Diện Chính

Cửa sổ chính gồm ba phần:

1. **Bảng Điều Khiển (Trái)**
   - Bật/Tắt ATO
   - Đặt mục tiêu dừng
   - Kiểm tra sức khỏe DCS
   - Chế độ jogging

2. **Hiển Thị Chính (Giữa)**
   - Vị trí tàu thực thời
   - Vận tốc, gia tốc
   - Trạng thái ATP/ATO
   - Thẻ nhận dạng di chuyển

3. **Phân Tích (Phải)**
   - Đồ thị đường cong P, W, B
   - Lịch sử tàu
   - Phân tích hiệu suất

### Điều Khiển Cơ Bản

```python
# Bật ATO
train.ato_mode = "ON"

# Đặt dừng tại vị trí 1000m
train.target_stop_m = 1000.0
train.commanded_stop = True

# Chế độ jogging (điều chỉnh vị trí)
train.jog_mode = "REQUESTED"
train.jog_distance_m = 10.0
train.jog_direction = "FORWARD"

# Kiểm tra trạng thái
if train.stopped and train.reported_speed < 0.1:
    print("Tàu đã dừng")
```

### Tải Kịch Bản Tùy Chỉnh

Chỉnh sửa CONFIG/scenario_loader.py:

```python
DEFAULT_SCENARIO_PATH = "DOCS/your_scenario.yaml"
```

Hoặc chạy với kịch bản khác:

```bash
python run.py --scenario DOCS/custom_scenario.yaml
```

## Kiến Trúc Hệ Thống

### Cấu Trúc Thư Mục

```
CBTC_SIM/
├── CONFIG/
│   ├── config.py               # Thông số cơ bản
│   └── scenario_loader.py       # Tải kịch bản YAML
│
├── SUBSYSTEMS/
│   ├── train.py                # Mô hình tàu
│   ├── physics.py              # Tính toán vật lý
│   ├── atp.py                  # Automatic Train Protection
│   ├── ato.py                  # Automatic Train Operation
│   ├── zc.py                   # Zone Controller
│   ├── dcs.py                  # Dispatch Control System
│   ├── signalling.py           # Tín hiệu bảo vệ
│   ├── control_common.py       # Thông số chung
│   ├── core_engine.py          # Engine chính
│   ├── runtime.py              # Vòng lặp chạy
│   └── communication/
│       ├── messages.py
│       ├── rasta.py            # Giao thức RASTA
│       ├── opcua.py            # Giao thức OpcUA
│       └── transport.py
│
├── GUI/
│   ├── main_gui.py             # Ứng dụng GUI
│   ├── panels/
│   │   ├── control_panel.py
│   │   ├── analytics_panel.py
│   │   ├── train_panel.py
│   │   └── ...
│   └── widgets/
│
├── OPERATION/
│   └── headway_manager.py      # Quản lý khoảng cách
│
├── DOCS/
│   └── default_scenario.yaml   # Kịch bản mặc định
│
└── run.py                      # Điểm vào
```

### Các Hệ Thống Con Chính

1. **Train (SUBSYSTEMS/train.py)**
   - Mô phỏng tàu vật lý
   - Quản lý vị trí, vận tốc, gia tốc
   - Tích hợp ATP, ATO, DCS

2. **Physics (SUBSYSTEMS/physics.py)**
   - Tính toán gia tốc từ lực
   - Lực kéo, phanh, cản chạy
   - Cập nhật vị trí và vận tốc

3. **ATP (SUBSYSTEMS/atp.py)**
   - Tính phong bì tốc độ an toàn
   - Đường cong P, W, SBI, EBI
   - Quản lý phanh và độ trễ

4. **ATO (SUBSYSTEMS/ato.py)**
   - Tạo mục tiêu vận hành
   - Điều khiển kéo/phanh
   - Dừng ga chính xác

5. **Zone Controller (SUBSYSTEMS/zc.py)**
   - Cấp thẻ nhận dạng di chuyển (MA)
   - Bảo vệ chống va chạm
   - Quản lý mục tiêu dừng

6. **DCS (SUBSYSTEMS/dcs.py)**
   - Trung tâm lưu chuyển
   - Giám sát sức khỏe mạng

### Cấu Hình

**CONFIG/config.py** - Thông số vật lý:

```python
TRAIN_LENGTH_M = 60.0
TRAIN_MASS_KG = 291600.0
MAX_SPEED_KMH = 80.0
A_MAX = 1.2              # Gia tốc m/s²
A_BRAKE = 1.12           # Giảm tốc phanh
A_EMERGENCY = 1.25       # Phanh khẩn cấp
BRAKE_BUILDUP_S = 1.5    # Thời gian xây dựng phanh
DT = 0.1                 # Bước thời gian
```

**DOCS/default_scenario.yaml** - Kịch bản:

```yaml
name: Cấu hình tuyến mặc định

display:
  track_min_m: -220.0
  track_max_m: 2000.0
  block_mode: moving_block

train_defaults:
  length_m: 60.0
  mass_kg: 291600.0
  drive_mode: ATO
  max_ato_speed_kmh: 70.0

radio_access_points:
  - id: RAP_01
    start_m: -300.0
    end_m: 900.0

localization:
  balises:
    - id: BL_0000
      pos_m: 0.0
```

## Các Tính Năng Chính

### 1. ATP (Automatic Train Protection)

Hệ thống bảo vệ chính đảm bảo an toàn giao thông:

- Tính toán phong bè tốc độ an toàn dựa trên vị trí và MA
- Quản lý đường cong tốc độ: P, W, SBD, EBD
- Xử lý phanh dịch vụ và phanh khẩn cấp
- Tính toán độ trễ phanh thực tế (1.5 giây)
- Điều chỉnh theo gradient đường ray
- Quản lý lề an toàn động

### 2. ATO (Automatic Train Operation)

Hệ thống vận hành tàu tự động:

- Tạo mục tiêu vận hành không vi phạm ATP
- Bộ điều khiển PID cho điều khiển tốc độ
- Dừng chính xác tại trạm (±0.5m)
- Hỗ trợ coasting và hồ sơ leo dốc

### 3. Train Model (Mô Hình Tàu)

Mô phỏng chi tiết của tàu vật lý:

- Tính toán lực kéo, phanh, cản chạy
- Mô hình chuyển động Newton (F = ma)
- Tính toán cản chạy polynomial: a_resist = A + B*v + C*v²
- Quản lý gradient: F_gradient = m*g*sin(θ)
- Hỗ trợ các khối lượng khác nhau
- Giới hạn jerk (0.75 m/s³)

### 4. Zone Controller (ZC)

Điều khiển vùng:

- Cấp (MA) cho tàu
- Tính toán bảo vệ chống va chạm
- Quản lý mục tiêu dừng
- Hỗ trợ chế độ moving_block

### 5. Communication (Truyền thông)

Truyền thông an toàn tàu-cơ sở hạ tầng:

- RASTA protocol: truyền thông an toàn với CRC
- OpcUA: giao thức tự động hóa
- Báo cáo vị trí định kỳ 
- Báo cáo sự kiện khi tốc độ thay đổi 
- Quản lý phạm vi Radio Access Points (RAP)
- Hỗ trợ nhiều điểm tiếp cận radio

### 6. GUI (Giao Diện Người Dùng)

Hiển thị trực quan và điều khiển:

- Bảng điều khiển tương tác
- Hiển thị thời gian thực vị trí/tốc độ
- Đồ thị đường cong ATP
- Lịch sử sự kiện
- Quản lý kịch bản

## Mô Hình Vật Lý

### Phương Trình Chuyển Động

Gia tốc tàu dựa trên lực Newton:

a = (F_traction - F_brake - F_resistance) / m

Trong đó:
- F_traction: Lực kéo từ động cơ
- F_brake: Lực phanh (dịch vụ hoặc khẩn cấp)
- F_resistance: Lực cản (chạy + gradient)
- m: Khối lượng tương đương

### Lực Cản

Mô hình polynomial:

F_resistance = m * (A + B*v + C*v²) + m*g*sin(θ)

Hệ số cản:
- A = 0.012 m/s²
- B = 0.00035 s/m
- C = 0.00002 1/m
- θ = gradient đường ray

### Mô Hình Phanh

Lực phanh tăng dần theo thời gian:

F(t) = F_max * (1 - e^(-t/τ))

Với τ = 1.5s (BRAKE_BUILDUP_S)

Điều này mô phỏng chính xác thời gian xây dựng lực phanh thực tế.

## Ví Dụ Sử Dụng

### Tạo và Điều Khiển Tàu

```python
from SUBSYSTEMS.train import Train

# Tạo tàu
train = Train(
    id="T001",
    length_m=60.0,
    mass_kg=291600.0,
    max_speed_kmh=80.0,
    drive_mode="ATO"
)

# Đặt vị trí ban đầu
train.reported_pos = 0.0
train.reported_speed = 0.0

# Bật ATO
train.ato_mode = "ON"

# Đặt mục tiêu dừng
train.target_stop_m = 1000.0
train.commanded_stop = True

# Chế độ jogging
train.jog_mode = "REQUESTED"
train.jog_distance_m = 5.0
train.jog_direction = "FORWARD"
```

### Kiểm Tra Trạng Thái ATP

```python
# Gọi ATP engine
atp_result = atp_engine.compute(train, track_profile)

# Kiểm tra kết quả
print(f"Tốc độ điều khiển: {atp_result.control_speed} m/s")
print(f"Đường cong P: {atp_result.curves['P']} m/s")
print(f"Khoảng cách phanh: {atp_result.actual_distance_to_stop} m")
print(f"Chế độ: {atp_result.curve_mode}")
```

### Tải Kịch Bản Tùy Chỉnh

```python
from CONFIG.scenario_loader import load_scenario

# Tải kịch bản
scenario = load_scenario("DOCS/my_scenario.yaml")

# Truy cập cấu hình
display = scenario['display']
trains_cfg = scenario['train_defaults']
raps = scenario['radio_access_points']
```


## Tiêu Chuẩn Tham Khảo

- EN 50159: Truyền thông an toàn trong hệ thống lưu lượng
- EN 50128: Phần mềm cho hệ thống lưu lượng
- EN 50126: Bảo vệ điều khiển tàu
- IEC 61508: Chức năng an toàn hệ thống

---

Phiên Bản: 1.2.0 | Ngày Cập Nhật: 12/06/2026 | Dự Án: CBTC_SIM
