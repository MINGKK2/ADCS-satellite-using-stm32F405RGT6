#include "telemetry_uart.h"
#include "usart.h"
#include <string.h>
#include <ctype.h>
#define RX_DMA_SIZE 256U
#define RX_RING_SIZE 8192U
#define FRAME_SIZE 1024U
#define CONTROL_SLOTS 16U
typedef struct {uint16_t length;uint8_t data[FRAME_SIZE];} Frame;
static uint8_t rx_dma[RX_DMA_SIZE],rx_ring[RX_RING_SIZE],inflight[FRAME_SIZE];
static volatile uint32_t rx_head,rx_tail;
static uint16_t dma_position;
static Frame controls[CONTROL_SLOTS],streams[3];
static uint32_t control_head,control_tail;
static volatile uint8_t tx_busy,rx_restart,estop_pending;
static volatile uint8_t rx_loss;
static uint8_t started;
static TelemetryUartStats stats;
static char urgent_line[20];
static uint8_t urgent_length,urgent_discard;
static uint32_t lock(void){uint32_t p=__get_PRIMASK();__disable_irq();return p;}
static void unlock(uint32_t p){__set_PRIMASK(p);}
__weak void TelemetryUART_EstopIRQ(void) {}
static void urgent_byte(uint8_t b) {
  if(b=='\r')return;
  if(b=='\n'){
    urgent_line[urgent_length]=0;
    if(!urgent_discard && (!strcmp(urgent_line,"ESTOP") || !strcmp(urgent_line,"EMERGENCY_STOP"))){
      estop_pending=1;TelemetryUART_EstopIRQ();
    }
    urgent_length=0;urgent_discard=0;return;
  }
  if(b<32||b>126||urgent_length>=sizeof(urgent_line)-1U){urgent_discard=1;return;}
  if(!urgent_discard)urgent_line[urgent_length++]=(char)toupper(b);
}
static void received(uint16_t end) {
  if(end>RX_DMA_SIZE)return;
  while(dma_position!=end) {
    if(dma_position==RX_DMA_SIZE){dma_position=0;if(end==RX_DMA_SIZE)break;}
    uint8_t b=rx_dma[dma_position++];
    stats.rx_bytes++;urgent_byte(b);
    if(rx_head-rx_tail<RX_RING_SIZE)rx_ring[rx_head++%RX_RING_SIZE]=b;
    else {stats.rx_dropped++;rx_loss=1;}
  }
}
HAL_StatusTypeDef TelemetryUART_Init(void) {
  memset(&stats,0,sizeof(stats));rx_head=rx_tail=0;dma_position=0;
  control_head=control_tail=0;memset(streams,0,sizeof(streams));
  tx_busy=0;estop_pending=0;urgent_length=urgent_discard=0;rx_restart=0;rx_loss=0;started=1;
  HAL_StatusTypeDef status=HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_dma,RX_DMA_SIZE);
  if(status!=HAL_OK){rx_restart=1;stats.uart_errors++;}
  return status;
}
int TelemetryUART_Send(const void *data,size_t length,int priority,unsigned stream) {
  if(!started||!data||!length||length>FRAME_SIZE){stats.tx_dropped++;return 0;}
  Frame *frame;
  if(priority) {
    if(control_head-control_tail>=CONTROL_SLOTS){stats.tx_dropped++;return 0;}
    frame=&controls[control_head%CONTROL_SLOTS];
  } else {
    if(stream>=3)return 0;
    frame=&streams[stream];
    if(frame->length)stats.tx_dropped++; /* latest sample wins, no stale backlog */
  }
  memcpy(frame->data,data,length);frame->length=(uint16_t)length;
  if(priority)control_head++;
  return 1;
}
size_t TelemetryUART_Read(uint8_t *data,size_t capacity) {
  size_t n=0;
  while(n<capacity && rx_tail!=rx_head){data[n++]=rx_ring[rx_tail%RX_RING_SIZE];__DMB();rx_tail++;}
  return n;
}
int TelemetryUART_TakeEstop(void) {
  uint32_t p=lock();int pending=estop_pending;
  if(pending){estop_pending=0;rx_tail=rx_head;}
  unlock(p);return pending;
}
int TelemetryUART_TakeRxLoss(void){uint32_t p=lock();int lost=rx_loss;rx_loss=0;unlock(p);return lost;}
void TelemetryUART_Service(void) {
  /* Only called from main context; ISR callbacks never start new work. */
  if(!started)return;
  if(rx_restart) {
    /* An RX UART error must not cancel an unrelated TX DMA frame. */
    if(huart1.RxState==HAL_UART_STATE_READY) {
      dma_position=0;
      if(HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_dma,RX_DMA_SIZE)==HAL_OK){rx_restart=0;stats.rx_restarts++;}
    }
  }
  if(tx_busy || huart1.gState!=HAL_UART_STATE_READY)return;
  Frame *frame=0;int control=0;
  if(control_tail!=control_head){frame=&controls[control_tail%CONTROL_SLOTS];control=1;}
  else for(unsigned i=0;i<3;i++)if(streams[i].length){frame=&streams[i];break;}
  if(!frame)return;
  uint16_t length=frame->length;memcpy(inflight,frame->data,length);
  uint32_t p=lock();tx_busy=1;
  HAL_StatusTypeDef status=HAL_UART_Transmit_DMA(&huart1,inflight,length);
  if(status!=HAL_OK){tx_busy=0;stats.tx_errors++;}
  unlock(p);
  if(status==HAL_OK){stats.tx_bytes+=length;frame->length=0;if(control)control_tail++;}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *u,uint16_t end) {if(u->Instance==USART1)received(end);}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *u) {if(u->Instance==USART1)tx_busy=0;}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u) {
  if(u->Instance!=USART1)return;
  stats.uart_errors++;
  rx_loss=1;urgent_discard=1;urgent_length=0;
  if(u->ErrorCode&HAL_UART_ERROR_DMA){tx_busy=0;stats.tx_errors++;}
  rx_restart=1;
}
TelemetryUartStats TelemetryUART_Stats(void){uint32_t p=lock();TelemetryUartStats s=stats;unlock(p);return s;}
