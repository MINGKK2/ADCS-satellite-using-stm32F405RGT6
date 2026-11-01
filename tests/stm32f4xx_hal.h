#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
#define __weak __attribute__((weak))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define __DMB() ((void)0)
#define USART1 ((void*)0x40011000)
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_ERROR_DMA 16U
typedef enum{HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct{void *Instance;unsigned gState,RxState,ErrorCode;} UART_HandleTypeDef;
typedef struct{int unused;} DMA_HandleTypeDef;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*,uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);
void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
#endif
