#include "telemetry_uart.h"
#include "usart.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
UART_HandleTypeDef huart1={.Instance=USART1};
DMA_HandleTypeDef hdma_usart1_rx,hdma_usart1_tx;
static uint8_t *dma_rx,*dma_tx;static uint16_t dma_size,tx_size,pos;static unsigned stops,restarts;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){(void)u;dma_rx=p;dma_size=n;pos=0;restarts++;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){u->gState=HAL_UART_STATE_BUSY_TX;dma_tx=p;tx_size=n;return HAL_OK;}
void TelemetryUART_EstopIRQ(void){stops++;}
static void complete(void){huart1.gState=HAL_UART_STATE_READY;HAL_UART_TxCpltCallback(&huart1);}
static void rx(const uint8_t *p,size_t n){
  for(size_t i=0;i<n;i++){dma_rx[pos++]=p[i];if(pos==dma_size/2||pos==dma_size)HAL_UARTEx_RxEventCallback(&huart1,pos);if(pos==dma_size)pos=0;}
  HAL_UARTEx_RxEventCallback(&huart1,pos?pos:dma_size);
}
int main(void){
  assert(TelemetryUART_Init()==HAL_OK);
  char frame[]="SEQ=1\r\n";
  assert(TelemetryUART_Send(frame,strlen(frame),0,0));frame[4]='9';
  assert(TelemetryUART_Send("ACK CMD=STATUS\r\n",16,1,0));
  TelemetryUART_Service();assert(tx_size==16&&!memcmp(dma_tx,"ACK CMD=STATUS\r\n",16));
  complete();TelemetryUART_Service();assert(!memcmp(dma_tx,"SEQ=1\r\n",7));
  assert(TelemetryUART_Send("NEW\r\n",5,0,0));assert(!memcmp(dma_tx,"SEQ=1\r\n",7));complete();
  TelemetryUART_Service();assert(!memcmp(dma_tx,"NEW\r\n",5));complete();
  assert(TelemetryUART_Send("OLD\n",4,0,0));assert(TelemetryUART_Send("LATEST\n",7,0,0));
  assert(TelemetryUART_Stats().tx_dropped==1);TelemetryUART_Service();assert(!memcmp(dma_tx,"LATEST\n",7));complete();
  uint8_t data[3000],out[3000];for(unsigned i=0;i<sizeof(data);i++)data[i]=(uint8_t)(i%26+'A');
  rx(data,sizeof(data));assert(TelemetryUART_Read(out,sizeof(out))==sizeof(out));assert(!memcmp(data,out,sizeof(out)));
  /* Duplicate idle callback at the same position must not duplicate bytes. */
  HAL_UARTEx_RxEventCallback(&huart1,pos?pos:dma_size);assert(!TelemetryUART_Read(out,sizeof(out)));
  rx((uint8_t*)"\nNOT_ESTop\n",11);assert(!stops);
  rx((uint8_t*)"EST",3);assert(!stops);rx((uint8_t*)"OP\r\n",4);assert(stops==1);
  assert(TelemetryUART_TakeEstop());assert(!TelemetryUART_TakeEstop());assert(!TelemetryUART_Read(out,sizeof(out)));
  uint8_t bad[200];memset(bad,'X',sizeof(bad));rx(bad,sizeof(bad));rx((uint8_t*)"ESTOP\n",6);assert(stops==1);
  rx(data,3000);rx(data,3000);rx(data,3000);assert(TelemetryUART_Stats().rx_dropped>0);assert(TelemetryUART_TakeRxLoss());
  huart1.ErrorCode=1;huart1.RxState=HAL_UART_STATE_READY;HAL_UART_ErrorCallback(&huart1);TelemetryUART_Service();
  assert(restarts==2&&TelemetryUART_Stats().uart_errors==1);
  /* Ground station absent: no RX bytes and even a stuck TX completion may
     discard telemetry, but must never block the caller or invent an ESTOP. */
  assert(TelemetryUART_Init()==HAL_OK);huart1.gState=HAL_UART_STATE_READY;
  assert(TelemetryUART_Send("FIRST\n",6,0,0));TelemetryUART_Service();
  for(unsigned i=0;i<60000;i++){
    assert(TelemetryUART_Send("OFFLINE\n",8,0,0));TelemetryUART_Service();
    assert(!TelemetryUART_Read(out,sizeof(out))&&!TelemetryUART_TakeEstop());
  }
  assert(TelemetryUART_Stats().tx_dropped==59999);
  complete();TelemetryUART_Service();assert(tx_size==8&&!memcmp(dma_tx,"OFFLINE\n",8));complete();
  rx((uint8_t*)"\nESTOP\r\n",8);assert(TelemetryUART_TakeEstop());
  puts("PASS offline UART: 60000 service cycles without ground RX or TX completion, bounded latest queue; explicit ESTOP still works on reconnection");
  puts("PASS UART: priority/DMA buffer lifetime, latest-frame queue, circular RX wrap, duplicate callbacks, urgent ESTOP, overflow/error counters and restart");
}
