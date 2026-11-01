# Khôi phục bản điều khiển trước commissioning Yaw — 2026-11-01

## Mốc khôi phục

Khôi phục từ `cascade-backup-20261031`, bản lưu trước khi thay EKF bằng complementary filter và chuyển sang bộ điều khiển cascade cần commissioning RPM. Đây là mốc trước các thử nghiệm PWM/Yaw gần đây, không phải bản cascade đang đặt gain bằng 0.

- EKF quaternion + gyro bias + accel + la bàn được khôi phục.
- PID tư thế trực tiếp xuất PWM: **Kp = 28, Ki = 2.4, Kd = 8** cho cả ba trục.
- Chu kỳ tư thế/PID 5 ms; encoder cập nhật 50 ms, 25 xung/vòng.
- PWM có độ lớn 0–255, PWM tối thiểu 0; dấu lệnh điều khiển chiều quay.
- Encoder trở về vai trò đo tốc độ/giám sát quá tốc và mất xung của bản cũ. Không phải phản hồi góc thân và không có vòng RPM cascade đang hoạt động.
- Khôi phục cả bảo vệ encoder cũ; không xóa fault liên tục để ép motor chạy.
- La bàn lỗi tạm thời không tự tắt PID khi IMU còn hợp lệ; không ép góc đo về target.
- Bỏ các module C++/MOTOR_TEST/auto-calib bằng bánh đà của bản thử khỏi build đang hoạt động.
- Khởi động với PID cả ba trục **tắt**, PWM bằng 0. Không tự chạy bánh đà khi nạp.
- Viewer được khôi phục cùng mốc, giữ mô hình CAD 3D và sửa xuống dòng log lỗi.

Các gain trên là giá trị của bản được khôi phục, **không phải kết quả nhận dạng/tuning mới**. Không kết luận ADCS đã ổn định trên phần cứng từ kết quả build.

## Ngoại lệ nhỏ để giữ hiệu chuẩn

`calibration_store.c` có thêm bộ đọc tương thích bản ghi v2 đã được firmware thử nghiệm lưu. Với ma trận từ trường đường chéo, phép chuyển sang ba hệ số scale của bản cũ là tương đương, giữ nguyên bias gyro mới nhất và hệ số accel/mag. Bản chụp journal gần nhất có v1 sequence 1/2 và v2 sequence 3; kiểm thử xác nhận lấy đúng bias gyro sequence 3.

Không hạ ma trận có hệ số ngoài đường chéo hoặc hiệu chuẩn chỉ 2D thành một bộ scale 3D. Những bản ghi không tương thích bị loại; các bản ghi hợp lệ cũ vẫn còn. Bộ ghi tiếp tục dùng định dạng v1, append-only, CRC/commit cuối, không tự xóa sector khi đầy. Linker `STM32F405RGTX_FLASH_CAL.ld` dành riêng sector 11: `0x080E0000–0x080FFFFF`.

## Các kiểm tra đã chạy

- Build Debug toàn bộ với GCC ARM 14.3: không warning/compile/link error.
- ELF: text 118968 B, data 1960 B, BSS 51200 B; load kết thúc `0x0801D868`, không chạm sector calib.
- SHA-256 ELF: `610C55BA97FB97700C5359B4419866AF23CA1614BDB7B7E121A5ED1443B0BDE5`.
- Test chạy các hàm thực trong source: quaternion, dấu/saturation/anti-windup PID, PPR/wrap/fault/overspeed encoder, covariance, EKF tổng hợp 120 giây, mất la bàn 6 giây và hồi phục không tắt PID.
- UART DMA: RX wrap, queue, ưu tiên ESTOP, bộ đệm TX, thống kê lỗi/mất dữ liệu.
- Parser và telemetry: lệnh phân mảnh/lỗi/quá dài, phục hồi đồng bộ; worst-case 7891.4 B/s tại 115200 baud; các dòng fast/slow/diag đều có CRLF.
- Hiệu chuẩn: CRC, mất điện ở mọi word ghi, journal đầy không xóa, v1/v2 đường chéo, bản chụp Flash thực tế.
- Viewer: góc đo tách target, dữ liệu cũ/mất/trùng gói, cảnh báo la bàn, calib Flash, mô hình CAD 24 phần và checksum.

**Chưa xác minh trong lần khôi phục này:** nạp lên board, telemetry radio sau nạp, đáp ứng cơ khí/motor hoặc giữ tư thế thực tế. Không có phép thử motor tự động trong quy trình khôi phục.

## Kết nối giữ nguyên

| Chức năng | STM32 | Module |
|---|---|---|
| Telemetry TX | PA9 / USART1_TX | C3 GPIO20 RX |
| Telemetry RX | PA10 / USART1_RX | C3 GPIO21 TX |
| I2C SCL / SDA | PB6 / PB7 | MPU6050, GY-511, BMP280, INA219, AHT30 |
| Roll PWM / DIR | PA6 / PC4, PC5 | L298N #1 ENA / IN1, IN2 |
| Pitch PWM / DIR | PA7 / PB12, PB13 | L298N #1 ENB / IN3, IN4 |
| Yaw PWM / DIR | PB0 / PB1, PB8 | L298N #2 ENA / IN1, IN2 |
| Encoder Roll / Pitch / Yaw | PA8 / PC6 / PA0 | OUT một kênh, mức 3.3 V |

UART 115200, 8N1, không flow control, GND chung. Chỉ nối một UART C3 để tránh lặp dữ liệu. S3 dùng COM23 trên máy hiện tại. GPS/camera chưa có driver thực; viewer nhận trạng thái chưa cấu hình.

## Đối chiếu sau khi nạp

1. Motor ngắt nguồn công suất, STM32/C3/S3 có nguồn logic. Kết nối viewer COM23 115200.
2. Xác nhận `PID=0,0,0`, `PWM3=0,0,0`, `ATK_u3=28000,2400,8000`, `CAL_SOURCE=FLASH` nếu journal còn hợp lệ.
3. Chờ dữ liệu IMU hợp lệ; quay thân bằng tay và giữ góc, kiểm tra góc đo không chạy theo target. Kiểm tra trạng thái la bàn và tuổi dữ liệu riêng.
4. Chỉ bật điều khiển sau khi gá phù hợp; trước mắt chỉ Yaw tự do. Phép thử điều khiển/motor cần một phiên riêng, không tự kích hoạt bởi việc khôi phục.

Ví dụ lệnh: `STATUS\r\n`, `SET_ATTITUDE,0,0,10\r\n`, `PID,YAW,1\r\n`, `ESTOP\r\n`. Lệnh bật PID có thể bị từ chối nếu đang ESTOP/IMU chưa hợp lệ; không dùng ví dụ này như script tự chạy.
