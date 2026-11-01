#include "command_parser.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static void feed(const char *s,uint32_t now){CommandParser_Feed((const uint8_t*)s,strlen(s),now);}
static ParsedCommand pop(void){ParsedCommand c;assert(CommandParser_Pop(&c));return c;}
int main(void) {
  CommandParser_Init();
  feed("SET_ATT",0);feed("ITUDE,1.5,-2.25,90\r\nSTATUS\nPID,YAW,0\n",5);
  ParsedCommand c=pop();assert(c.type==USB_COMMAND_SET_ATTITUDE&&c.values[0]==1.5f&&c.values[1]==-2.25f&&c.values[2]==90);
  assert(pop().type==USB_COMMAND_STATUS);c=pop();assert(c.type==USB_COMMAND_PID&&c.axis==2&&c.enabled==0);
  assert(!CommandParser_Pop(&c));
  feed("CTRL_MODE,ATTITUDE\nCTRL_MODE,WHEEL_SPEED\nSET_WHEEL_RPM,100,200,-300\nWHEEL_PID,YAW,0.1,0.2,0\nRPM_LIMIT,1000,2000,3000\nENCODER_CLEAR\n",6);
  assert(pop().type==USB_COMMAND_CONTROL_ATTITUDE);assert(pop().type==USB_COMMAND_CONTROL_WHEEL_SPEED);
  c=pop();assert(c.type==USB_COMMAND_SET_WHEEL_RPM&&c.values[2]==-300);
  assert(pop().type==USB_COMMAND_SET_WHEEL_PID);assert(pop().type==USB_COMMAND_SET_RPM_LIMIT);assert(pop().type==USB_COMMAND_ENCODER_CLEAR);
  feed("MOTOR_TEST,YAW,20,1000\nATT_PID,YAW,1,1,1\nCAL_MAG_CANCEL\nSET_WHEEL_RPM,30001,0,0\n",8);
  for(int k=0;k<4;k++)assert(pop().type==USB_COMMAND_UNKNOWN);
  const char *commands[]={"ZERO_YAW","CAL GYRO","CAL MAG","CAL ACCEL START","CAL ACCEL CAPTURE","CAL ACCEL CANCEL","MODE,WORKING","MODE,CHARGING","CAMERA,CAPTURE","ESTOP","ESTOP_CLEAR","PID,ALL,1"};
  const UsbCommand_t types[]={USB_COMMAND_ZERO_YAW,USB_COMMAND_CAL_GYRO,USB_COMMAND_CAL_MAG,USB_COMMAND_CAL_ACCEL_START,USB_COMMAND_CAL_ACCEL_CAPTURE,USB_COMMAND_CAL_ACCEL_CANCEL,USB_COMMAND_MODE_WORKING,USB_COMMAND_MODE_CHARGING,USB_COMMAND_CAMERA_CAPTURE,USB_COMMAND_ESTOP,USB_COMMAND_ESTOP_CLEAR,USB_COMMAND_PID};
  for(unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);i++){feed(commands[i],10);feed("\n",10);assert(pop().type==types[i]);}
  feed("PID,ALL,2\nPID,YAW,1x\nSET_ATTITUDE,0,90,0\nSET_ATTITUDE,NaN,0,0\n",20);
  for(unsigned i=0;i<4;i++)assert(pop().type==USB_COMMAND_UNKNOWN);
  char longline[200];memset(longline,'A',sizeof(longline));CommandParser_Feed((uint8_t*)longline,sizeof(longline),30);
  feed("ESTOP\nSTATUS\n",35);assert(pop().type==USB_COMMAND_STATUS);assert(!CommandParser_Pop(&c));
  feed("SET_ATTITUDE,",40);feed("1,2,3\nSTATUS\n",1000);assert(pop().type==USB_COMMAND_STATUS);
  feed("PID,YAW,",1010);CommandParser_DiscardPartial();feed("1\nSTATUS\n",1020);assert(pop().type==USB_COMMAND_STATUS);
  for(int i=0;i<40;i++){feed("STATUS\n",1030);}
  int count=0;while(CommandParser_Pop(&c))count++;
  assert(count==32&&CommandParser_Stats().queue_dropped==8);
  printf("PASS parser: fragmented/batched commands, full viewer command set, bounds, invalid/long line recovery, timeout, queue counters\n");
}
