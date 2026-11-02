# Ghi đáp ứng tư thế và xuất biểu đồ báo cáo

Mở `http://127.0.0.1:8765/attitude-viewer.html#attitudeReport`.
File giao diện: `work/attitude-viewer.html`; logic ghi/phân tích ở
`attitude-report-core.js`, hiển thị/xuất ảnh ở `attitude-report.js`.

## Ghi một phép thử

1. Kết nối S3/COM23 như trước, kiểm tra dữ liệu đo hợp lệ. Chọn trục theo gá thử
   thực tế; việc ghi dữ liệu không bật PID hoặc gửi lệnh motor.
2. Chọn **Biểu đồ / báo cáo**, đặt tên phép thử, ví dụ `Yaw 0 đến 90 độ`.
3. Bấm **Bắt đầu ghi** trước khi gửi góc mục tiêu bằng phần điều khiển sẵn có.
   Cần ghi được các mẫu trước/sau bước target để tính thời gian đáp ứng.
4. Quan sát đường liền (góc EKF thực nhận) và đường đứt (target trong telemetry).
   Khi đủ dữ liệu, bấm **Dừng ghi**. Có thể ghi tiếp nhưng ranh giới tạm dừng
   được đánh dấu là khoảng mất tính liên tục.
5. Chọn trục, loại đồ thị và khoảng thời gian; có thể zoom trên đồ thị.
6. Xuất **CSV** để giữ toàn bộ dữ liệu, **PNG** hoặc **SVG** để chèn báo cáo,
   **Kết quả JSON** để giữ tiêu chí và chỉ số phân tích. Nếu trình duyệt hỏi
   nơi lưu hoặc yêu cầu xác nhận tải xuống, hoàn tất thao tác lưu của trình duyệt.

Ảnh có nền trắng, tên phép thử, trục/đơn vị, chú giải góc đo/target, firmware,
nguồn UART hay CSV và các chỉ số. PNG mặc định rộng 3200 pixel; SVG là hình
vector với các trace 2D. Ảnh/JSON dùng phạm vi thời gian đang chọn/zoom; CSV
luôn giữ toàn phiên. CSV UTF-8 có BOM, dấu phân cách là dấu phẩy.

## Chỉ số và cách đọc

- Chỉ phân tích đoạn target không đổi cuối cùng trong khoảng đang chọn, từng
  trục riêng. Đổi target không sửa lại góc đo đã ghi.
- Sai số Euler là `wrap180(target - measured)`; khác sai số quaternion hệ thân
  của PID. RMSE tính trên mẫu tư thế hợp lệ trong đoạn đó.
- Sai số cuối là trung bình góc tròn của sai số ở 1 giây cuối, tránh lỗi trung
  bình 179° và -179° thành 0°. Không tự coi đây là sai số xác lập nếu dữ liệu
  còn đang chuyển động.
- Vượt target tính theo độ trong chiều bước target quan sát được; không phải
  phần trăm overshoot hoặc đại lượng đã nhận dạng mô hình.
- Điều kiện mặc định là sai số trong ±2° liên tục ít nhất 1 giây. Có thể thay
  hai ngưỡng này trên UI. Một mẫu lỗi, mất đoạn hoặc ra khỏi dải ở cuối sẽ làm
  mất kết luận đã giữ dải trước đó.
- Thời gian vào dải chỉ hiện khi quan sát được bước thay đổi target; bắt đầu
  từ mẫu đầu thấy target mới, không phải thời điểm chính xác driver nhận lệnh.
  Nếu chỉ ghi được trạng thái đã ở target, app có thể báo đang trong dải nhưng
  không tự đưa ra settling time 0 giây.
- Kết luận chỉ có hiệu lực trong đoạn dữ liệu đã ghi. Góc đo là đầu ra EKF;
  cần đối chiếu phép đo độc lập để xác nhận góc vật lý tuyệt đối hoặc drift.

## Dữ liệu và tính liên tục

CSV giữ góc gốc R/P/Y, target, Euler error, body error, gyro, PID output,
PWM yêu cầu/thực, RPM, yêu cầu PID/quyền xuất/bit chặn, ESTOP, encoder presence,
tuổi IMU/diagnostic/mag, firmware/gain và thời gian MCU/PC.

Chỉ ghi các FAST frame được parser hiện có chấp nhận, đọc dữ liệu từ chính
dòng raw, không dùng cache target/góc để điền trường thiếu. Mẫu `ATT_VALID=0`,
IMU quá 20 ms hoặc thiếu trường góc/target được giữ trong CSV với cờ invalid,
không dùng cho chỉ số hoặc nối đường góc đo. Sequence gap, khoảng MCU >250 ms,
khoảng nhận PC >500 ms và ghi tiếp sau tạm dừng đều làm ngắt đường liên tục.

MCU đổi epoch/khởi động lại sẽ dừng phiên, giữ dữ liệu cũ để xuất và yêu cầu
tạo phiên mới. Wrap uptime uint32 được hỗ trợ. Encoder một kênh vẫn chỉ là
RPM magnitude; Roll chưa lắp sẽ không được vẽ như RPM đo bằng 0.

Dữ liệu nằm trong RAM trình duyệt. Xuất CSV trước khi F5/đóng trang. Giới hạn
36.000 mẫu (~30 phút ở 20 Hz); đạt giới hạn chỉ dừng ghi, không tác động PID.
Đồ thị cập nhật tối đa khoảng 2 lần/giây, giữ cực trị khi giảm điểm vẽ; thống kê
và CSV dùng toàn bộ mẫu, không dùng tập điểm giảm của đồ thị.

**Mở CSV** hoặc mục **Dán CSV thay vì chọn file** để vẽ lại định dạng CSV do
trang xuất. Nhập CSV được gắn nhãn rõ, không tạo telemetry giả cho mô hình 3D
hoặc các nút điều khiển. Dừng ghi trước khi thay tập dữ liệu bằng CSV.

## Thư viện và xác minh

Plotly basic 4.2.0 được lưu cục bộ trong `vendor/plotly`, có LICENSE và metadata
package/integrity; không cần CDN khi dùng trang. Chỉ bật các nút zoom/pan/reset
cục bộ; không bật nút chia sẻ cloud. API ảnh theo tài liệu
[Plotly static export](https://plotly.com/javascript/static-image-export/).

Các test `test-attitude-report.cjs` và `test-dashboard.cjs` đã qua: đáp ứng giả
lập có overshoot 5° và thời gian vào dải quan sát 1,8 s; thiếu mẫu, invalid,
reset/wrap, không bịa target từ cache, round-trip CSV, sai số qua ±180° và giữ
đỉnh khi giảm điểm vẽ. Đường nhập CSV bằng cách dán đã thử trên trình duyệt;
automation chọn file bị chặn bởi quyền extension, không có thay đổi quyền này.

Kiểm tra UI đã dùng fixture `report-fixtures/synthetic-yaw-step.csv`, gắn nhãn
**MÔ PHỎNG KIỂM THỬ**, không phải kết quả phần cứng. PNG tạo được 3200×1700 và
đọc/giải mã được; SVG tạo được và parse XML hợp lệ. Browser trên máy giữ bước
hoàn tất lưu file tải xuống, nên chưa xác nhận thao tác lưu cuối của người dùng.
Không gửi lệnh PID/motor, không mở COM và không thay firmware trong công việc này.
