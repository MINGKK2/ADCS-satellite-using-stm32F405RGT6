# STM32F405 UART telemetry V14

Build ngày 19/11/2026 từ project `E:\STM_Coding\STM32CubeIDE`.
Đã tạo firmware; chưa nạp hoặc đo UART trên board.

## Dữ liệu gửi

USART1 PA9 TX / PA10 RX, 115200 baud, 8N1. Chu kỳ gửi 50 ms (20 Hz).
Mỗi gói có 52 byte: header CCSDS telemetry 12 byte và payload 10 float32 (40 byte).
Không có CSV, CRLF, ACK/NACK, banner hoặc bản tin chẩn đoán trong chiều UART TX.
UART RX, xử lý lệnh và ESTOP vẫn dùng mã cũ; phản hồi dạng chữ đã được chặn.

| Thứ tự | Offset tính từ đầu gói | Tên trường phía nhận | Nguồn dữ liệu | Đơn vị |
|---|---:|---|---|---|
| 1 | 12 | Temperature | (BMP280 °C + AHT30 °C) / 2 | °C |
| 2 | 16 | Pressure | BMP280 | Pa |
| 3 | 20 | Roll | EKF | độ |
| 4 | 24 | Pitch | EKF | độ |
| 5 | 28 | Yaw | EKF | độ |
| 6 | 32 | Voltage | **Dòng điện INA219** | **A** |
| 7 | 36 | RelativeAltitude | Độ cao tương đối từ BMP280 | m |
| 8 | 40 | Latitude | GPS | độ |
| 9 | 44 | Longitude | GPS | độ |
| 10 | 48 | RemainingEnergy | **Điện áp bus INA219** | **V** |

Nhiệt độ BMP280 đang lưu ở centi-°C; AHT30 đang lưu ở milli-°C. Cả hai được
đổi về °C trước khi cộng và chia 2. Nếu một trong hai không hợp lệ hoặc cũ,
Temperature là IEEE-754 NaN, không lấy cảm biến còn lại thay cho trung bình.
Các trường khác cũng gửi NaN nếu nguồn tương ứng không hợp lệ/cũ; tọa độ cần GPS fix.
Float32 làm tròn tọa độ về độ chính xác biểu diễn của float32.

## Header và cấu hình phía nhận cần đối chiếu

Chưa được cung cấp typedef/header cấu hình và Message ID của mission phía nhận.
Do đó đây là bản build theo các giả định tường minh sau, chưa xác nhận tương thích
nhị phân với mọi cấu hình `CFE_MSG_TelemetryHeader_t`:

- Primary header CCSDS v1: 6 byte, big-endian.
- Packet ID / Message ID mặc định tạm: **0x0881** (APID 0x081, telemetry, có secondary header).
- Sequence flags: unsegmented; sequence count 14 bit tăng sau khi enqueue thành công.
- Packet Length: **45**, tương ứng tổng gói 52 byte theo quy tắc length = total − 7.
- Secondary header: 4 byte seconds + 2 byte binary fractional seconds, big-endian.
- Timestamp lấy từ **uptime STM32**, không phải giờ cFE đã đồng bộ hoặc mission epoch.
- Payload: IEEE-754 float32 **little-endian**, không padding giữa các trường.
- Không CRC/checksum hoặc UART framing phụ ngoài header và length nêu trên.

Đổi `TELEMETRY_SENSOR_MSG_ID` và `TELEMETRY_PAYLOAD_BIG_ENDIAN` trong
`Core/Inc/telemetry_protocol.h` hoặc bằng compiler define nếu phía nhận dùng giá trị khác.
Nếu mission dùng header/epoch khác thì phải sửa phần serializer trước khi sử dụng.

Đường ESP32-C3/S3 cũ trong bản repository đã khảo sát có xử lý theo dòng văn bản.
Firmware STM32 mới phát binary không có kết thúc dòng: **không coi bridge/viewer cũ
là đã tương thích**. Phía nhận cần đọc binary theo header/length hoặc dùng bridge
truyền byte nguyên vẹn; firmware radio không được thay đổi trong yêu cầu này.

## Kiểm chứng

- Force rebuild GNU ARM 14.3.rel1 thành công, không thấy warning/error trong build.log.
- Host tests kiểm tra 52 byte, header, thứ tự và đơn vị 10 trường, trung bình nhiệt độ,
  dòng điện âm, thiếu từng cảm biến nhiệt độ, NaN, tắt mọi event TX, chu kỳ gửi,
  wrap sequence 14 bit, wrap tick và enqueue thất bại.
- Chạy cả payload little-endian và big-endian với Message ID override.
- ELF load segments kết thúc trước 0x080E0000; vùng calibration sector 11 được chừa.
- Đối chiếu hash nguồn: chỉ sửa `Core/Src/telemetry_protocol.c` và
  `Core/Inc/telemetry_protocol.h`. Không sửa EKF/PID, chân/baud UART, driver hoặc linker.
- Chưa flash, chưa kiểm tra dữ liệu cảm biến thực hoặc kết nối cFE trên phần cứng.

## Tệp trong gói

- `STM32CubeIDE.bin`, `.hex`, `.elf`, `.map`: cùng một lần build mới.
- `source/`: hai tệp nguồn đã sửa.
- `tests/test_cfe_packet.c`: kiểm thử host của serializer.
- `build.log`, `manifest.json`: log build và checksum các tệp.

Các tệp BIN/HEX mới cũng đã được tạo tại `E:\STM_Coding\STM32CubeIDE\Debug`.
Không dùng mass erase khi cần giữ journal hiệu chuẩn ở sector 11.
