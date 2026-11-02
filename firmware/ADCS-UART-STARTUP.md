# STM32 UART startup fix — AXIS_FRAME_RP_UART_V2

Đã build, chạy bộ test, nạp USB DFU và verify; sector11 calib trước/sau giống từng byte. Đã xác minh trên phần cứng qua USB debug C3/COM34: 50233 byte/132 bản tin hoàn chỉnh trong6s, đúng FW=AXIS_FRAME_RP_UART_V2, STATUS chuyển qua UART và ACK trả về, IMU hợp lệ với tuổi mẫu<=5ms, SENSOR_ERR=0x00, CAL_SOURCE=FLASH, PID/PWM0. Chưa chạy motor hoặc kiểm chứng chuyển động Roll/Pitch sau nạp; chưa xác nhận lại toàn tuyến ESP-NOW/S3/HTML.

## Source

main USER CODE1 chọn SCB->VTOR=FLASH_BASE, DSB/ISB và mở global IRQ trước HAL_Init. USER CODE2 gọi MX_DMA_Init rồi MX_USART1_UART_Init rõ ràng trước TelemetryUART_Init. SystemInit trước đó không có USER_VECT_TAB_ADDRESS nên không tự cố định VTOR khi được ROM GO; đây là điểm được gia cố, không phải bằng chứng đã đo được nguyên nhân trước sửa.

USART1 vẫn PA9 TX→C3 GPIO20 RX; PA10 RX←C3 GPIO21 TX; GND chung; 115200 8N1 không flow-control. RX DMA2 Stream2/Channel4, TX Stream7/Channel4 và NVIC handler giữ nguyên. Không dùng UART blocking trong vòng điều khiển, không đổi firmware hai ESP.

Giữ sửa frame Roll/Pitch, gyro bias chuyển đổi đúng khi đọc/ghi Flash, Yaw và gain28/2.4/8. Giữ motor Roll PA6+PC4/5, Pitch PA7+PB12/13, Yaw PB0+PB1/8. Encoder Pitch PC6, Yaw PA0; Roll chưa lắp, không bỏ bảo vệ encoder.

## Kiểm thử phần mềm

Build toàn bộ không warning/error; bộ test parser/protocol/DMA RX-TX/ESTOP/60000 vòng transport nghẽn, 600s giữ PID không lệnh, EKF/quaternion/antiwindup, frame tay phải và bias calibration journal/UI đều qua. ELF load end0x0801DDF4 không chạm sector11.
