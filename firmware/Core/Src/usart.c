/* CubeMX-compatible USART1 peripheral module. No telemetry logic here. */
#include "usart.h"
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx,hdma_usart1_tx;
void MX_USART1_UART_Init(void) {
  huart1.Instance=USART1;
  huart1.Init.BaudRate=115200;
  huart1.Init.WordLength=UART_WORDLENGTH_8B;
  huart1.Init.StopBits=UART_STOPBITS_1;
  huart1.Init.Parity=UART_PARITY_NONE;
  huart1.Init.Mode=UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart1.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart1)!=HAL_OK)Error_Handler();
}
void HAL_UART_MspInit(UART_HandleTypeDef *u) {
  if(u->Instance!=USART1)return;
  GPIO_InitTypeDef gpio={0};
  __HAL_RCC_USART1_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin=GPIO_PIN_9|GPIO_PIN_10;gpio.Mode=GPIO_MODE_AF_PP;
  gpio.Pull=GPIO_PULLUP;gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;gpio.Alternate=GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA,&gpio);
  hdma_usart1_rx.Instance=DMA2_Stream2;hdma_usart1_rx.Init.Channel=DMA_CHANNEL_4;
  hdma_usart1_rx.Init.Direction=DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc=DMA_PINC_DISABLE;hdma_usart1_rx.Init.MemInc=DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;hdma_usart1_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode=DMA_CIRCULAR;hdma_usart1_rx.Init.Priority=DMA_PRIORITY_HIGH;
  hdma_usart1_rx.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
  if(HAL_DMA_Init(&hdma_usart1_rx)!=HAL_OK)Error_Handler();
  __HAL_LINKDMA(u,hdmarx,hdma_usart1_rx);
  hdma_usart1_tx.Instance=DMA2_Stream7;hdma_usart1_tx.Init.Channel=DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction=DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc=DMA_PINC_DISABLE;hdma_usart1_tx.Init.MemInc=DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;hdma_usart1_tx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode=DMA_NORMAL;hdma_usart1_tx.Init.Priority=DMA_PRIORITY_LOW;
  hdma_usart1_tx.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
  if(HAL_DMA_Init(&hdma_usart1_tx)!=HAL_OK)Error_Handler();
  __HAL_LINKDMA(u,hdmatx,hdma_usart1_tx);
  HAL_NVIC_SetPriority(USART1_IRQn,6,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
}
void HAL_UART_MspDeInit(UART_HandleTypeDef *u) {
  if(u->Instance!=USART1)return;
  __HAL_RCC_USART1_CLK_DISABLE();HAL_GPIO_DeInit(GPIOA,GPIO_PIN_9|GPIO_PIN_10);
  HAL_DMA_DeInit(u->hdmarx);HAL_DMA_DeInit(u->hdmatx);HAL_NVIC_DisableIRQ(USART1_IRQn);
}
