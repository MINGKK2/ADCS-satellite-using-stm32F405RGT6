# Giữ tư thế độc lập trạm mặt đất

Firmware: AUTONOMOUS_HOLD_V1. Đã build/test và nạp USB DFU, verify thành công; toàn bộ sector calib trước/sau giống nhau. Ứng dụng đã xuất hiện COM21. Chưa xác nhận telemetry firmware qua S3/C3 vì hai cổng không hiện trên máy; chưa thử motor/rút nguồn S3 thực tế.

Source không có timeout radio trực tiếp tắt PID. Các nhánh reset controller/EKF và mất chu kỳ của bản cũ có thể xóa PID hoặc đổi target; đã sửa giữ yêu cầu PID và target trong RAM. Không kết luận những nhánh này là nguyên nhân thực tế lúc rút S3 khi chưa đo trên phần cứng.

- Sau khi đặt góc/bật từng PID, STM32 tự chạy vòng EKF/PID khi S3 tắt. Không cần heartbeat hoặc giữ kết nối trình duyệt.
- Kết nối lại không tự đổi góc hay bật/tắt PID. Viewer lấy lại trạng thái STM32.
- Chu kỳ trễ tạm thời không xóa yêu cầu PID/target; lỗi nghiêm trọng vẫn chặn đầu ra/phục hồi cảm biến.
- PID/target không được lưu như lệnh tự chạy qua mất nguồn STM32. Khởi động STM32 vẫn PID OFF, PWM0.
- ESTOP, lỗi IMU, encoder và giới hạn RPM tiếp tục bảo vệ motor. Không ép góc đo về target.
- P=28, I=2.4, D=8; EKF, calib Flash, chân PWM/DIR, USART1 PA9/PA10 và DMA giữ nguyên. Không thêm lại chức năng nạp không dây.
- S3/C3 không cần đổi firmware cho thay đổi STM32 này; UART của STM32 không dùng flow control và TX dùng hàng đợi DMA có giới hạn.

Kiểm thử phần mềm: 600 giây mô phỏng điều khiển không có lệnh mới, recovery không thay target; 60000 lần service khi RX im lặng/TX không hoàn tất; hồi kết nối xử lý ESTOP đúng. Bộ kiểm thử EKF/PID/encoder/Flash/UI đều qua; load kết thúc 0x0801DCA0, ngoài sector calib. Không chạy motor thật.

Kiểm thử trên mô hình sau nạp: cấp nguồn riêng STM32/C3 và motor theo gá thử; đặt target và bật PID cần dùng; ghi góc/target/PID/PWM/uptime, tắt riêng S3, bật lại sau 30–60 giây và so sánh target/PID, uptime không reset. Đây chưa phải phép đo giữ tư thế đã hoàn tất.
