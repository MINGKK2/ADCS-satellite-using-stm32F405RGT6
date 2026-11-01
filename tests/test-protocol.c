#include "telemetry_protocol.h"
#include "telemetry_uart.h"
#include "command_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static size_t bytes,maxlen[3];static unsigned frames[3],bad;
int TelemetryUART_Send(const void *p,size_t n,int priority,unsigned stream) {
  if(!p||!n||n>1024){bad++;return 0;}
  const char *s=p;assert(s[n-2]=='\r'&&s[n-1]=='\n');
  if(!priority){bytes+=n;frames[stream]++;if(n>maxlen[stream])maxlen[stream]=n;}
  return 1;
}
TelemetryUartStats TelemetryUART_Stats(void){TelemetryUartStats s={0};return s;}
CommandStats CommandParser_Stats(void){CommandStats s={0};return s;}
int main(void) {
  TelemetryData d={0};d.attitude_valid=1;d.who=0x70;d.charging=1;d.sv=7;
  for(unsigned i=0;i<3;i++){d.angle[i]=-180000;d.target[i]=-180000;d.gyro[i]=-250000;
    d.rpm_milli[i]=30000000;d.pid_output[i]=d.pwm[i]=d.requested_pwm[i]=-255;
    d.count[i]=2147483647;d.encoder_rejected[i]=4294967295U;d.error[i]=-180000;
    d.p_term[i]=d.i_term[i]=d.d_term[i]=-2147483647;d.rpm_limit[i]=30000000;
    d.wheel_kp[i]=d.wheel_ki[i]=d.wheel_kd[i]=1000000000;}
  d.yaw_rate=-250000;d.mag_age_ms=4294967295U;d.control_late=4294967295U;
  d.pressure_pa=110000;d.altitude_mm=-1000000;d.voltage_mv=12600;d.current_ma10=-32000;
  TelemetryProtocol_Init();
  for(unsigned t=10;t<=10000;t+=10){d.uptime_ms=t;TelemetryProtocol_Service(&d);}
  printf("protocol: fast=%u slow=%u diag=%u; max bytes=%zu/%zu/%zu; total rate=%.1f B/s\n",frames[0],frames[1],frames[2],maxlen[0],maxlen[1],maxlen[2],bytes/10.0);
  assert(!bad&&frames[0]==200&&frames[1]==20&&frames[2]==10);
  assert(bytes/10.0<11000.0); /* worst-case digits; below 11520 B/s UART capacity */
}
