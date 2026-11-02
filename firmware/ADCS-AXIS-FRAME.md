# Sửa ánh xạ Roll/Pitch theo đo thực tế

Firmware AXIS_FRAME_RP_V1. Build/test đã qua; đã nạp USB DFU và verify, calib sector11 trước/sau giống từng byte. Ứng dụng xuất hiện COM21. Chưa kiểm tra telemetry và chuyển động trục sau nạp vì S3/COM23 chưa hiện trên máy.

Đã đo bằng tay với motor ngắt nguồn, PID OFF: nghiêng Roll thực tế làm Pitch cũ tăng khoảng56deg; nghiêng Pitch thực tế làm Roll cũ tăng khoảng53deg. Người dùng chọn giữ chiều Yaw và quy ước tay phải.

## Thuật toán

Áp dụng phép quay cơ sở R=[[0,1,0],[-1,0,0],[0,0,1]] vào accel/gyro/mag và vector trục Yaw đã nhận dạng. R có determinant+1, giữ +Z/Yaw và độ dài/cross-product. Góc Euler được tính lại từ EKF/quaternion; không chỉ đổi nhãn hoặc tráo hai số Euler, đặc biệt ở góc lớn.

Raw accel/mag calibration vẫn theo trục cảm biến cũ. Gyro bias khi đọc Flash được chuyển sang body mới, khi lưu chuyển ngược về encoding lịch sử. Vì encoding/đơn vị lưu không đổi nên journal v1/v2-diagonal cũ tiếp tục hợp lệ và rollback còn dùng được. Test roundtrip giữ nguyên byte và không ghi thêm bản ghi trùng.

## Đấu dây được người dùng xác nhận

| Trục | PWM | DIR | Encoder |
|---|---|---|---|
| Roll | PA6 | PC4, PC5 | Chưa lắp |
| Pitch | PA7 | PB12, PB13 | PC6 / TIM8 |
| Yaw | PB0 | PB1, PB8 | PA0 / TIM2 |

Giữ nguyên GPIO/timer, gain P=28 I=2.4 D=8 và dấu motor đã cấu hình. Không tự đổi chiều mô-men khi chưa có phép đo motor; chưa chạy motor hoặc xác nhận giữ tư thế thực tế sau sửa.

Firmware báo ENC_INSTALLED3=0,1,1; viewer hiển thị Roll chưa lắp encoder thay vì số RPM0 như một phép đo. Không vô hiệu hóa bảo vệ encoder/mất xung/quá tốc. Vì Roll chưa có encoder, bảo vệ hiện tại có thể chặn đầu ra Roll khi yêu cầu motor mà không có xung; xử lý chính sách đó cần quyết định riêng, không xóa fault để ép chạy.

Giữ nguyên chức năng giữ PID/target độc lập S3 và khởi động PID OFF/PWM0. Không thêm lại updater không dây.

## Kiểm tra

Bộ test hàm C thực xác nhận pose Roll/Pitch đã đo ra đúng góc và sai số PID, gyro/mag cùng hệ, trục Yaw giữ đúng, quaternion/ESKF/antiwindup/encoder/UART DMA/ESTOP/calibration Flash/UI đều qua. ELF load kết thúc0x0801DDE8, ngoài sector11 calib.

Sau nạp, motor tiếp tục ngắt nguồn: kiểm tra FW=AXIS_FRAME_RP_V1, BODY_MAP=Y_NEGX_Z, ENC_INSTALLED3=0,1,1. Quay Roll bằng tay phải đổi chủ yếu Roll; Pitch phải đổi chủ yếu Pitch, chiều dương theo cơ sở tay phải đã chọn. Kiểm tra chiều phản lực từng motor trên gá phù hợp là phép thử riêng, chưa được thực hiện.
