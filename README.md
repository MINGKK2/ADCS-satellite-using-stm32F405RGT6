# CubeSat ADCS — STM32F405RGT6

Bản hiện tại: **`AXIS_FRAME_RP_PITCH_KP_V4`**, đã nạp USB DFU và verify ngày
**2026-11-01**. Kp của Roll/Pitch/Yaw đã giảm một lần từ 28 xuống
**28 / 2,4 = 11,6667**; Ki = 2,4 và Kd = 8. Đã đảo riêng chiều điều khiển Pitch.
Chi tiết thay đổi và giới hạn kiểm chứng: [ADCS V4](docs/ADCS-V4-20261101.md).

## Source và bản build

- `firmware/`: project STM32CubeIDE, `.ioc`, HAL/CMSIS/USB, linker chừa sector 11 cho calib.
- `viewer/`: giao diện Web Serial, mô hình CAD 3D và thư viện Three.js cục bộ.
- `radio/`: cầu telemetry **RADIO_ONLY_V1** C3/S3 hiện đang sử dụng; không có chức năng nạp STM32 không dây.
- `artifacts/stm32-v4/`: ELF/BIN/HEX/map V4 và manifest SHA-256. Các thư mục artifact khác là mốc lịch sử.
- `tests/`, `tools/`: kiểm thử thuật toán thực, DMA/parser/protocol, Flash calib và viewer.

## Điều khiển và phần cứng

STM32F405RGT6, HSE 8 MHz / CPU 168 MHz; ba motor DC chổi than qua L298N.
EKF dùng gyro/accel/la bàn; PID tư thế xuất PWM trực tiếp, độ lớn 0–255.
Không có Auto tune hay vòng RPM cascade đang hoạt động. Encoder một kênh đo
độ lớn RPM, phục vụ giám sát/giới hạn tốc độ và bảo vệ; không đo góc thân hoặc
chiều quay độc lập. Gain hiện tại chưa được xác nhận là bộ gain tuning hoàn tất.

| Trục | PWM | IN1 | IN2 | Encoder | Dấu điều khiển motor |
|---|---|---|---|---|---|
| Roll | PA6 / TIM3_CH1 | PC4 | PC5 | PA8, **chưa lắp** | +1 |
| Pitch | PA7 / TIM3_CH2 | PB12 | PB13 | PC6 | **+1**, trước đây −1 |
| Yaw | PB0 / TIM3_CH3 | PB1 | PB8 | PA0 | −1 |

Encoder cấu hình 25 xung/vòng; chu kỳ tư thế/PID đặt 5 ms, đo encoder 50 ms.
Hệ trục thân tay phải: `(X_mới,Y_mới,Z_mới)=(Y_cũ,−X_cũ,Z_cũ)`, áp dụng đồng bộ
cho accel/gyro/la bàn; giữ trục Yaw. Không đổi chân motor Roll/Pitch theo tên góc.
Flash gyro bias vẫn giữ định dạng hệ cũ và được chuyển hệ khi load/save.

MPU6050 (WHO 0x68/0x70), GY-511, BMP280, INA219 và AHT30 dùng I2C1 PB6/PB7.
GPS/camera chưa có driver phần cứng; viewer không tự tạo dữ liệu cho các mục này.
PID ON/OFF thuộc người vận hành; mất liên lạc/la bàn không tự xóa yêu cầu PID hoặc
target. Phản hồi không hợp lệ vẫn có thể chặn đầu ra bảo vệ, hiển thị qua
`PID_ACTIVE`/`PID_BLOCK3`. Boot mặc định PID OFF/PWM 0; ESTOP luôn ưu tiên.

## Telemetry và viewer

STM32 **PA9 TX → C3 GPIO20 RX**, **PA10 RX ← C3 GPIO21 TX**, GND chung,
logic 3,3 V, USART1 **115200 8N1** → ESP-NOW → S3 → USB **COM23**.
Chỉ UART0 C3 làm nguồn telemetry. Xem [radio/README.md](radio/README.md).

```powershell
python -m http.server 8765 --bind 127.0.0.1 --directory viewer
```

Mở `http://127.0.0.1:8765/attitude-viewer.html`, chọn COM của S3, 115200 baud.
Chỉ một ứng dụng được mở COM cùng lúc. Viewer đánh dấu dữ liệu cũ/không hợp lệ,
tách góc đo, target, sai số hệ thân, PID output, PWM thực và RPM từng bánh đà.

Protocol `KEY=value`, kết thúc CRLF; fast đặt 20 Hz, slow 2 Hz, diag 1 Hz.
Góc mdeg, gyro mdps, RPM milli-RPM, PWM có dấu. Hệ số V4 được báo qua
`ATK_u3=11667,2400,8000` theo quy ước milli-unit hiện có của trường này.
Lệnh kết thúc LF/CRLF: `STATUS`, `SET_ATTITUDE,0,0,10`, `PID,YAW,1`,
`PID,ALL,0`, `ESTOP`; phản hồi `ACK CMD=...` hoặc `NACK CMD=...`.

## Build và kiểm thử

1. STM32CubeIDE: Import Existing Projects, chọn `firmware`, Build Debug.
2. Giữ linker `STM32F405RGTX_FLASH_CAL.ld` cho Debug/Release; sector 11 dành riêng calib.
3. Sau khi CubeIDE sinh `Debug/makefile`, force rebuild bằng script dưới đây.
   Script cũng tạo BIN/HEX mới từ ELF, tránh dùng ảnh cũ còn trong thư mục Debug.

```powershell
powershell -ExecutionPolicy Bypass -File tools/build-stable.ps1
powershell -ExecutionPolicy Bypass -File tools/test-stable.ps1
```

Công cụ đã dùng: CubeIDE 2.1.1 / GNU ARM 14.3.rel1; GCC host, Python 3, Node.js.
Scripts có tham số đường dẫn nếu công cụ cài ở nơi khác. Kiểm thử radio riêng
được hướng dẫn trong `radio/README.md`. Test host không mở COM hoặc chạy motor.
Một số timer/GPIO được khởi tạo trong USER CODE; đọc source trước khi Generate Code.

## Nạp và phạm vi xác minh

Ngắt nguồn công suất motor, vào USB DFU. Sao lưu toàn bộ sector 11 trước nạp,
nạp ELF và verify, không mass erase; đọc lại sector 11 để so SHA-256.
V4 đã verify thành công, sector calib 128 KiB trước/sau giống hệt nhau.
Không đưa bản dump calib của board lên Git. Manifest chỉ ghi hash kiểm chứng.

Build/kiểm thử phần mềm đã qua. Hai ESP radio-only đã nạp và verify; từng đo được
telemetry khoảng 19 Hz và STATUS ACK hai chiều. Tuy nhiên có lần rút USB C3 thì
mất radio dù đèn còn sáng; nguồn riêng của C3 chưa được đo/xác minh dứt điểm.
**Sau nạp STM32 V4, S3 chỉ trả thống kê cục bộ, chưa nhận telemetry/STATUS ACK.**
Chưa kiểm chứng chiều mô-men Pitch mới hoặc độ ổn định giữ góc với gain V4 trên
phần cứng. Không kết luận ADCS ổn định chỉ vì build/verify hoặc có telemetry.

Các tài liệu trước V4 và các artifact cũ là lịch sử. Chức năng updater không dây
đã được bỏ khỏi source hiện tại; có thể tra lại ở commit `3405635` nếu cần nghiên
cứu. Nạp STM32 hiện dùng USB DFU. Mã HAL/CMSIS/USB và Three.js giữ nguyên giấy phép.
