#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>
typedef struct {uint32_t tx_bytes,rx_bytes,tx_dropped,rx_dropped,uart_errors,tx_errors,rx_restarts;} TelemetryUartStats;
HAL_StatusTypeDef TelemetryUART_Init(void);
void TelemetryUART_Service(void);
int TelemetryUART_Send(const void *data,size_t length,int priority,unsigned stream);
size_t TelemetryUART_Read(uint8_t *data,size_t capacity);
int TelemetryUART_TakeEstop(void);
int TelemetryUART_TakeRxLoss(void);
void TelemetryUART_EstopIRQ(void);
TelemetryUartStats TelemetryUART_Stats(void);
#endif
