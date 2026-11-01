#ifndef USART_H
#define USART_H
#include "main.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx,hdma_usart1_tx;
void MX_USART1_UART_Init(void);
#endif
