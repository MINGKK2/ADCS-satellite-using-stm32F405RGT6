#include "gps_uart.h"
#include "gps_receiver.h"
#include "usart.h"

#define GPS_RX_CAPACITY 1024U
#define GPS_SERVICE_BUDGET 256U

/* Interrupt is the producer, application service is the consumer. */
static uint8_t rx_buffer[GPS_RX_CAPACITY];
static volatile uint32_t write_index, read_index;
static volatile bool rx_lost;

void GpsUart_Start(void)
{
  GpsReceiver_Reset();
  write_index = read_index = 0;
  rx_lost = false;
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  /* Application-owned IRQ, below the USART1 ground link priority. */
  HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void GpsUart_ReceiveIRQ(void)
{
  uint32_t status = USART2->SR;
  const uint32_t errors = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;
  if (!(status & (USART_SR_RXNE | errors))) return;

  /* Reading SR then DR clears the F405 UART receive/error flags. */
  uint8_t byte = (uint8_t)USART2->DR;
  uint32_t write = write_index;
  if ((status & errors) || write - read_index >= GPS_RX_CAPACITY) {
    rx_lost = true;
    return;
  }
  rx_buffer[write & (GPS_RX_CAPACITY - 1U)] = byte;
  __DMB();
  write_index = write + 1U;
}

void GpsUart_Service(void)
{
  if (rx_lost) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    read_index = write_index;
    rx_lost = false;
    __set_PRIMASK(primask);
    GpsReceiver_Reset();
  }

  uint32_t now = HAL_GetTick();
  for (unsigned i = 0; i < GPS_SERVICE_BUDGET && !rx_lost; ++i) {
    uint32_t read = read_index;
    if (read == write_index) break;
    __DMB();
    uint8_t byte = rx_buffer[read & (GPS_RX_CAPACITY - 1U)];
    __DMB();
    read_index = read + 1U;
    GpsReceiver_Feed(byte, now);
  }
  if (rx_lost) GpsReceiver_Reset();
}
