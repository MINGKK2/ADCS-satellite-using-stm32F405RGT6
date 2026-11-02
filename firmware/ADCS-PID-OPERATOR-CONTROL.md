# PID do người dùng quyết định — 2026-11-01

## Lỗi tìm thấy trong source

`pid_enabled[]` bị xóa trong chuẩn hóa quaternion lỗi, reset controller, khởi tạo lại EKF, calib accel/la bàn/gyro và nhánh trễ vòng điều khiển >20 ms. Vì các nhánh phục hồi gọi lại reset, một sự cố tạm thời có thể biến yêu cầu PID ON thành OFF và đổi target sang tư thế mới. Parser ở trạng thái busy cũng từ chối cả lệnh PID OFF thông thường.

Tab đang mở còn giữ HTML cascade/complementary cũ trong khi firmware đã quay về EKF/PID trực tiếp. Mẫu cuối trên tab ghi `PID=1,1,1`, `PWM3=0,0,0` và đã cũ hơn 10 phút: bằng chứng đó không đồng nghĩa PID đã tắt. UI cần tách yêu cầu ON với quyền cấp đầu ra, và đánh dấu tuổi dữ liệu.

## Sửa đã thực hiện

- `pid_enabled[]` là **yêu cầu của người dùng**, không bị xóa bởi lỗi/quá trình phục hồi/calib. Lệnh PID và thao tác ESTOP/xóa ESTOP rõ ràng của người dùng là nơi thay đổi trạng thái này.
- Khởi động nguội/nạp firmware vẫn mặc định PID OFF, PWM 0; không lưu trạng thái ON để tự chạy motor sau reset.
- Yêu cầu ON có thể được chấp nhận trong khi cảm biến đang phục hồi, nhưng đầu ra vẫn chặn cho tới khi hợp lệ. ESTOP đang chốt sẽ từ chối bật PID. OFF vẫn được xử lý trong vòng phục hồi/busy; IRQ ESTOP giữ ưu tiên.
- Reset controller không đổi target đã đặt. EKF khi phục hồi giữ heading đo gần nhất và tham chiếu từ trường còn hợp lệ, không lấy heading từ target.
- Trễ 20–100 ms: giữ yêu cầu ON, tạm chặn đầu ra 100 ms và dùng dt thực cho EKF. Khoảng mất mẫu >100 ms vẫn yêu cầu phục hồi estimator, không xóa yêu cầu PID. Kiểm thử có wrap của tick 32-bit.
- Lưu calib xét motor/PWM yêu cầu đã dừng, không coi trạng thái ON được giữ là motor đang chạy. Điều này tránh làm mất chức năng lưu Flash khi calib trong trạng thái PID ON.
- Không thay gain: Kp=28, Ki=2.4, Kd=8. Không thêm autotune.

## Telemetry/UI

`PID=R,P,Y`: yêu cầu ON/OFF của người dùng. `PID_ACTIVE=R,P,Y`: controller được phép hoạt động theo health hiện tại, không phải đo điện áp chân PWM. `PID_BLOCK3=R,P,Y`: bitmask lý do đầu ra bị chặn.

| Bit | Lý do |
|---:|---|
| 1 | ESTOP |
| 2 | IMU lỗi hoặc mẫu cũ |
| 4 | EKF đang phục hồi |
| 8 | Calib/phục hồi đang busy |
| 16 | Fault encoder đã chốt |
| 32 | Giá trị RPM chưa hợp lệ |
| 64 | Giới hạn RPM |
| 128 | Tạm chặn do trễ chu kỳ |

Viewer giữ checkbox ON theo `PID`, hiển thị riêng “PID ON · đang điều khiển” hoặc “PID ON · đầu ra tạm chặn: …”. Dữ liệu chẩn đoán cũ không được ghi đè trạng thái block mới nhận trong bản tin fast. `PWM3` tiếp tục là lệnh đầu ra thực sau giới hạn, có thể bằng 0 khi PID ON.

Không vô hiệu hóa bảo vệ cảm biến, giới hạn RPM, fault encoder hoặc ESTOP để giả báo hệ vẫn giữ tư thế. PID ON với đầu ra bị chặn không được trình bày như đang điều khiển thực tế. Nếu motor vẫn dừng, cần đọc bitmask và dữ liệu hiện tại để xử lý đúng lỗi phần cứng/phản hồi.

## Kiểm thử

Host test chạy hàm thật trong source: PID ON/target tồn tại sau controller/EKF reset; heading đo 45° giữ 45° dù target 90°; pause 25 ms không reinit, khoảng >100 ms không xóa latch; wrap thời gian; các test EKF/quaternion/PID anti-windup/encoder/cảm biến cũ vẫn qua.

UI test: ON vẫn được check khi encoder fault chặn đầu ra, trạng thái hoạt động trở lại khi block hết mà không cần lệnh ON mới; góc đo vẫn tách target. UART DMA/ESTOP/parser/protocol/Flash journal tiếp tục được kiểm thử. Dòng firmware mới mang `FW=MANUAL_PID_V1`.

Nạp không dây cần cài cầu S3/C3 mới một lần qua USB, sau đó truyền file qua S3 → ESP-NOW → C3 và nạp ROM UART STM32; xem `radio/README.md` trong repo. Chưa được gọi là đã nạp nếu chưa có verify/readback và xác nhận firmware sau GO.
