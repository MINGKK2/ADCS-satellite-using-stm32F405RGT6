# CubeSat ADCS — STM32F405RGT6

Bản khôi phục ngày **2026-11-01**, trước đợt thử Yaw/PWM và chuyển sang cascade PID. Khôi phục EKF và PID tư thế trực tiếp xuất PWM, cùng viewer tương ứng. **Kp 28 / Ki 2.4 / Kd 8** cho Roll/Pitch/Yaw là gain lịch sử của bản được khôi phục, chưa phải gain đã nhận dạng/tuning hoàn tất.

## Nội dung

- `firmware/`: project STM32CubeIDE, HAL/CMSIS/USB middleware, `.ioc`, linker dành riêng sector 11 cho calib.
- `viewer/`: giao diện telemetry/điều khiển, mô hình CAD 3D, Three.js cục bộ và giấy phép đi kèm.
- `tests/`, `tools/`: regression test thuật toán thực, UART DMA, protocol, bộ nhớ calib và viewer.
- `artifacts/stable-debug/`: ELF Debug đã build, map và manifest SHA-256 để đối chiếu đúng bản.
- [Chi tiết khôi phục, kiểm thử và bảng nối chân](docs/STABLE-RESTORE-20261101.md).

## Phần cứng và điều khiển

STM32F405RGT6, HSE 8 MHz / CPU 168 MHz; ba motor DC chổi than qua hai L298N; encoder một kênh 25 xung/vòng. MPU6050 (chấp nhận WHO 0x68/0x70), GY-511, BMP280, INA219 và AHT30 trên PB6/PB7.

Chu kỳ tư thế/PID 5 ms; encoder 50 ms. Encoder của bản này phục vụ đo RPM, giới hạn quá tốc và bảo vệ mất xung. **Không có vòng RPM cascade đang hoạt động**, không coi encoder bánh đà là cảm biến góc thân và không coi dấu PWM là chiều quay đo được.

Luồng telemetry: STM32 USART1 **PA9 TX → C3 GPIO20 RX**, **PA10 RX ← C3 GPIO21 TX**, GND chung, 3.3 V, 115200 8N1 → ESP-NOW → S3 → USB. Chỉ nối một UART C3. Firmware C3/S3 không nằm trong snapshot STM32 này.

Boot mặc định PID tắt/PWM 0. Không có autotune hoặc MOTOR_TEST trong bản khôi phục. Camera/GPS chưa có driver phần cứng, được báo chưa cấu hình.

## Build

1. Mở STM32CubeIDE, **File → Import → General → Existing Projects into Workspace**, chọn thư mục `firmware` có `.project`.
2. Chọn **Debug**, Build Project. Bản đã xác minh dùng STM32CubeIDE 2.1.1 / GNU ARM 14.3.rel1.
3. Giữ linker `STM32F405RGTX_FLASH_CAL.ld` ở cả Debug/Release. Không đổi sang linker FLASH đầy 1 MiB: sector 11 lưu calib.
4. Sau khi IDE sinh thư mục `Debug`, có thể force rebuild bằng `powershell -ExecutionPolicy Bypass -File tools/build-stable.ps1` (truyền `-ToolchainBin`/`-MakeExe` nếu công cụ cài ở nơi khác).

Các timer PWM/encoder và một số cấu hình được khởi tạo trong USER CODE của `main.c`; đọc source trước khi Generate Code, không dựa riêng vào sơ đồ pin CubeMX. Gain/mapping đang dùng là của mô hình cụ thể này.

## Viewer

Từ thư mục repo:

```powershell
python -m http.server 8765 --bind 127.0.0.1 --directory viewer
```

Mở `http://127.0.0.1:8765/attitude-viewer.html` bằng trình duyệt hỗ trợ Web Serial, chọn COM của S3 (COM23 trên máy triển khai), 115200 baud. Không mở cùng COM bằng hai ứng dụng.

Telemetry là các trường `KEY=value` kết thúc CRLF; fast 20 Hz, slow 2 Hz, diag 1 Hz. Góc mdeg, gyro mdps, RPM milli-RPM, PWM có dấu; `PWM3` là lệnh đầu ra thực đã qua giới hạn. Ví dụ cú pháp, **không phải dữ liệu đo mới**:

```text
SEQ=1 T_ms=1000 R=0 P=0 Y=90000 TARGET_mdeg=0,0,0 PIDOUT3=0,0,0 REQPWM3=0,0,0 PWM3=0,0,0 ERPM3_m=0,0,0 PID=0,0,0 ATT_VALID=1
```

Lệnh kết thúc LF/CRLF: `STATUS`, `SET_ATTITUDE,0,0,10`, `PID,YAW,1`, `PID,ALL,0`, `ESTOP`. Phản hồi bắt đầu `ACK CMD=...` hoặc `NACK CMD=...`; xem parser/main.c để biết điều kiện chấp nhận. Không chạy ví dụ bật PID khi gá chưa phù hợp.

## Regression test

```powershell
powershell -ExecutionPolicy Bypass -File tools/test-stable.ps1
```

Cần GCC host, Python 3, Node.js. Script có tham số `-Compiler`, `-Python`, `-Node`; mặc định dùng các đường dẫn cài trên máy triển khai rồi tìm công cụ trong PATH. Test không truy cập cổng COM và không chạy motor. Kiểm thử EKF tổng hợp 120 s, PID/encoder, mất la bàn 6 s, DMA/ESTOP, parser, bandwidth, CRC/mất điện khi ghi calib, viewer và mô hình CAD.

## Nạp và hiệu chuẩn

Ngắt nguồn công suất motor, đưa STM32 vào DFU. Đọc/sao lưu toàn bộ sector 11 **trước khi nạp**; nạp ELF và verify, không mass erase. Đọc lại journal để so SHA-256 trước/sau. ELF cung cấp không có load segment trong sector 11. Calib Flash của từng board không đưa lên Git; các mặc định trong source là dữ liệu của mô hình gốc, cần hiệu chuẩn nếu dùng phần cứng khác.

Bộ đọc calib có ngoại lệ tương thích v2 đường chéo để giữ bias gyro mới nhất từ bản thử nghiệm. Ma trận đầy đủ hoặc calib chỉ Yaw 2D không được tự coi là calib 3D hợp lệ cho thuật toán cũ.

## Phạm vi xác minh

Build Debug không warning/error và regression test phần mềm đã qua. **Snapshot này chưa được xác nhận lại trên motor/gá thực sau rollback.** Không kết luận hệ ADCS ổn định chỉ từ build/telemetry. Giữ PID tắt sau nạp, kiểm tra dữ liệu đo rồi thử từng trục theo gá thực tế.

Mã HAL/CMSIS/USB và Three.js giữ thông báo bản quyền/giấy phép của từng bên trong thư mục tương ứng; không gán lại giấy phép cho mã của bên thứ ba.
