# Viewer cho Mission_SensorTlm V14

Mở server tại thư mục này:

```powershell
python -m http.server 8765 --bind 127.0.0.1
```

Truy cập `http://127.0.0.1:8765/attitude-viewer.html` bằng Chrome hoặc Edge.
Chọn Message ID đúng firmware (mặc định 0x0881), byte order payload (mặc định
little-endian), rồi Kết nối UART. Cổng luôn dùng 115200 baud, 8N1.

## Gói tin duy nhất được hỗ trợ

- Tổng 52 byte: primary header CCSDS 6 byte + timestamp 6 byte + 10 float32.
- Header big-endian, sequence flags unsegmented, packet-length field bằng 45.
- Timestamp là uptime MCU, không chuyển sang ngày giờ lịch.
- Payload bắt đầu offset 12; không nhận CSV hoặc các bản tin KEY=value cũ.
- Parser ghép lại gói chia nhỏ, tách nhiều gói trong một lần đọc, tìm lại header
  sau byte rác và chỉ giữ tối đa 52 byte trong bộ đệm ghép gói.
- Không có checksum trong firmware hiện tại; header/length không phát hiện được
  mọi lỗi truyền. Khớp cấu hình header và byte order là điều kiện bắt buộc.

| Thứ tự | Trường trong struct | Hiển thị | Đơn vị |
|---|---|---|---|
| 1 | Temperature | Nhiệt độ trung bình BMP280 và AHT30, tính ở STM32 | °C |
| 2 | Pressure | Áp suất | Pa |
| 3 | Roll | Góc Roll | độ |
| 4 | Pitch | Góc Pitch | độ |
| 5 | Yaw | Góc Yaw | độ |
| 6 | Voltage | **Dòng điện INA219** | **A** |
| 7 | RelativeAltitude | Độ cao tương đối | m |
| 8 | Latitude | Vĩ độ | độ |
| 9 | Longitude | Kinh độ | độ |
| 10 | RemainingEnergy | **Điện áp INA219** | **V** |

NaN/Infinity hiển thị dấu —. Sau 1,5 giây không có gói mới, trang đánh dấu dữ liệu
cũ và mô hình không hợp lệ. Mô hình giữ hình cuối; không giả lập chuyển động.
Tọa độ là float32 từ firmware nên hiển thị sáu số lẻ không tăng độ chính xác đo.
Không có chỉ số pin %, năng lượng còn lại, RPM, PWM, target phản hồi hoặc PID
đang bật vì chúng không tồn tại trong gói này.

## Lệnh điều khiển

Các nút gửi ASCII + CRLF: SET_ATTITUDE, PID, ZERO_YAW, MODE WORKING/CHARGING,
ESTOP/ESTOP_CLEAR, CAL_GYRO, CAL_MAG, CAL_ACCEL_START/CAPTURE/CANCEL.
Chỉ ghi “Đã gửi”; firmware không trả ACK/NACK hoặc tiến độ hiệu chuẩn trong gói
V14. Giá trị target nhập và yêu cầu PID trên form không được coi là trạng thái
đã xác nhận của STM32. ESTOP_CLEAR giữ PID OFF theo firmware.

## Radio và kết nối thật

Trang nhận **byte binary nguyên vẹn** từ cổng được chọn. Firmware bridge C3/S3
trong `radio/` vẫn xử lý theo dòng text nên cần được đổi hoặc thay bằng đường
truyền byte nguyên vẹn. Không sửa bridge/radio trong thay đổi viewer này.

## Kiểm thử

```powershell
node test-cfe-telemetry.cjs
node test-cfe-dashboard.cjs
```

Test dashboard cần package Playwright và Chromium hoặc Microsoft Edge.
Biến `ADCS_UI_SCREENSHOT_DIR` chọn thư mục lưu ảnh kiểm tra. Test dùng Web Serial
giả lập và dữ liệu tổng hợp, không mở COM hoặc gửi lệnh tới phần cứng thật.
Test cũ `test-dashboard.cjs` thuộc giao thức text V9, không dùng cho trang V14.
Các module báo cáo cũ vẫn lưu trong repo nhưng không được nạp bởi HTML hiện tại.
