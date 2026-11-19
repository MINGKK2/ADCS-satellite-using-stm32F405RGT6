# CubeSat ADCS — STM32F405RGT6

Bản source hiện tại: **AXIS_FRAME_RP_UART_CFE10_V14**, build ngày **19/11/2026**.
USART1 gửi gói nhị phân **52 byte** mỗi **50 ms**, ở **115200 baud / 8N1**.
Bản build mới đã qua kiểm thử phần mềm và kiểm tra vùng Flash hiệu chuẩn;
**chưa nạp hoặc xác minh UART/cFE trên phần cứng**.

## Dữ liệu telemetry

Payload có đúng 10 float32 theo thứ tự:

| Trường phía nhận | Dữ liệu gửi | Đơn vị |
|---|---|---|
| Temperature | (BMP280 + AHT30) / 2, sau khi đổi cùng đơn vị | °C |
| Pressure | BMP280 | Pa |
| Roll | EKF | độ |
| Pitch | EKF | độ |
| Yaw | EKF | độ |
| Voltage | **Dòng điện INA219** | **A** |
| RelativeAltitude | Độ cao tương đối BMP280 | m |
| Latitude | GPS | độ |
| Longitude | GPS | độ |
| RemainingEnergy | **Điện áp bus INA219** | **V** |

Nhiệt độ chỉ hợp lệ khi cả hai cảm biến còn hợp lệ và không quá hạn.
Thiếu dữ liệu gửi IEEE-754 NaN. UART TX không xen ACK/NACK, CAL, banner hoặc
diagnostic; RX và ESTOP vẫn giữ luồng xử lý hiện có.

**Cấu hình header chưa được đối chiếu mission phía nhận:** mặc định CCSDS v1,
primary header 6 byte và time 6 byte, Message ID tạm **0x0881**, payload
**little-endian**. Timestamp dùng uptime STM32, không phải thời gian cFE đã đồng bộ.
Chi tiết byte offset, các macro cấu hình và phạm vi kiểm chứng nằm trong
[đặc tả UART V14](docs/UART-CFE10-V14.md).

## Source và artifact

- `firmware/`: project CubeIDE đầy đủ; gồm baseline V13 đang dùng trên máy
  và thay đổi serializer V14. GPS trên USART2 PA2/PA3, 115200 baud.
- `artifacts/stm32-v14/`: ELF, BIN, HEX, map và manifest SHA-256 của lần build V14.
- `tests/test-cfe-protocol.c`: kiểm tra framing, thứ tự/đơn vị, trung bình nhiệt độ,
  NaN, sequence/tick wrap, queue rejection và không có event TX.
- `tools/test-current-uart.ps1`: kiểm thử serializer hai byte order, UART DMA,
  ESTOP và journal hiệu chuẩn.
- `viewer/`: giao diện Web Serial giải mã gói binary V14, hiển thị 10 trường,
  mô hình 3D theo Roll/Pitch/Yaw và gửi 12 nhóm lệnh ASCII. Xem
  [hướng dẫn viewer](viewer/README.md).
- `radio/`: cầu radio theo dòng text của các bản trước,
  **chưa chuyển sang truyền nguyên gói binary V14**.

Bridge/parser cũ không được coi là tương thích. Muốn dùng V14 qua radio cần đường
truyền byte nguyên vẹn hoặc parser binary theo header/length ở phía nhận.
Không có thay đổi firmware ESP32 hoặc chức năng nạp không dây trong bản này.

## Build và kiểm thử

Import `firmware` vào STM32CubeIDE, Build Debug lần đầu để sinh makefile, sau đó:

```powershell
powershell -ExecutionPolicy Bypass -File tools/build-stable.ps1
powershell -ExecutionPolicy Bypass -File tools/test-current-uart.ps1
python tools/check-calibration-elf.py firmware/Debug/STM32CubeIDE.elf
```

Công cụ build: CubeIDE 2.1.1 / GNU ARM 14.3.rel1; kiểm thử host dùng GCC.
Script build tái tạo BIN/HEX từ ELF. Giữ linker
`STM32F405RGTX_FLASH_CAL.ld`: sector 11 được dành cho hiệu chuẩn.
Không mass erase nếu cần giữ journal; chưa có thao tác flash trong lần phát hành này.

## Nền điều khiển và lịch sử

Baseline được đồng bộ từ project đang build tại máy, không lấy lại cấu hình V9.
Roll/Pitch: Kp = 28/2,4, Ki = 5, Kd = 8. Yaw: Kp = 9, Ki = 3,5, Kd = 8.
Các giá trị này không được khẳng định là bộ gain đã tuning hoàn tất.
Encoder một kênh đo độ lớn RPM; PID tư thế xuất PWM trực tiếp, mặc định OFF sau boot.

[README V9](docs/README-V9.md), các tài liệu V9/V4 và artifact cũ là lịch sử.
`tools/test-stable.ps1` và các test parser/protocol dạng text cũ phục vụ baseline
lịch sử, không phải bộ kiểm thử giao thức V14.
