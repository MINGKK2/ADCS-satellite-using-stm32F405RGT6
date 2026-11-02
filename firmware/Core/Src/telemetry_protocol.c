#include "telemetry_protocol.h"
#include "telemetry_uart.h"
#include "command_parser.h"
#include <stdio.h>
#include <string.h>
static uint32_t fast_tick,slow_tick,diag_tick,sequence;
static char line[1024];
#define V(a) (long)s->a[0],(long)s->a[1],(long)s->a[2]
#define U(a) (unsigned long)s->a[0],(unsigned long)s->a[1],(unsigned long)s->a[2]
#define B(a) s->a[0],s->a[1],s->a[2]
static const char *mag_states[]={"WAIT_REFERENCE","TRACKING","REJECTED","STALE","I2C_ERROR"};
static const char *mag_reasons[]={"NONE","FIELD_NORM","HORIZONTAL","INNOVATION","FILTER","OVERFLOW","NO_DATA","I2C"};
static void send(int n,unsigned stream) {
  if(n>0 && n<(int)sizeof(line))TelemetryUART_Send(line,(size_t)n,0,stream);
  else TelemetryUART_Send(0,0,0,stream); /* records a format overflow, never truncated UART lines */
}
void TelemetryProtocol_Init(void){fast_tick=slow_tick=diag_tick=sequence=0;}
void TelemetryProtocol_Service(const TelemetryData *s) {
  if((uint32_t)(s->uptime_ms-fast_tick)>=50U) {
    fast_tick=s->uptime_ms;
    int n=snprintf(line,sizeof(line),
      "SEQ=%lu T_ms=%lu R=%ld P=%ld Y=%ld TARGET_mdeg=%ld,%ld,%ld G_mdps=%ld,%ld,%ld YR=%ld PIDOUT3=%ld,%ld,%ld REQPWM3=%ld,%ld,%ld PWM3=%ld,%ld,%ld ERPM3_m=%ld,%ld,%ld PID=%u,%u,%u MODE=%s ESTOP=%u ATT_VALID=%u BERR_mdeg=%ld,%ld,%ld IMU_AGE_ms=%lu PID_ACTIVE=%u,%u,%u PID_BLOCK3=%u,%u,%u PID_OWNER=OPERATOR\r\n",
      (unsigned long)++sequence,(unsigned long)s->uptime_ms,V(angle),V(target),V(gyro),(long)s->yaw_rate,
      V(pid_output),V(requested_pwm),V(pwm),V(rpm_milli),B(pid),s->charging?"CHARGING":"WORKING",s->estop,s->attitude_valid,V(body_error),(unsigned long)s->imu_age_ms,B(pid_active),B(pid_block));
    if(n>1 && n<(int)sizeof(line))n=n-2+snprintf(line+n-2,sizeof(line)-(size_t)(n-2),
        " Q_u6=%ld,%ld,%ld,%ld\r\n",(long)s->quaternion[0],(long)s->quaternion[1],(long)s->quaternion[2],(long)s->quaternion[3]);
    send(n,0);
  }
  if((uint32_t)(s->uptime_ms-slow_tick)>=500U) {
    slow_tick=s->uptime_ms;
    int n=snprintf(line,sizeof(line),
      "SLOW=1 T_ms=%lu WHO=0x%02X A_mg=%ld,%ld,%ld M=%ld,%ld,%ld MRJ=%lu MGR=%lu,%lu AGR=%lu,%lu Q_u6=%ld,%ld,%ld,%ld BMP_TcC=%ld PPa=%lu ALT_mm=%ld INA_VmV=%ld INA_ImA10=%ld INA_PmW=%ld BAT_pct=%ld AHT_TmC=%ld RHmPct=%lu SV=0x%02X DT_us=%lu STILL=%u YAW_MAG_AGE_ms=%lu MAG_QUALITY=%u FUSION=%u SENSOR_ERR=0x%02lX CAMERA=NOT_CONFIGURED GPS_FIX=0 GPS_LAT_e7=0 GPS_LON_e7=0 GPS_SATS=0 CTRL=%s HOLD=FIXED_PID FW=AXIS_FRAME_RP_DYN_V9 HOLD_LINK=INDEPENDENT BODY_MAP=Y_NEGX_Z\r\n",
      (unsigned long)s->uptime_ms,s->who,V(accel),V(mag),(unsigned long)s->mag_rejected,
      (unsigned long)s->mag_accepted,(unsigned long)s->mag_rejected,(unsigned long)s->accel_accepted,(unsigned long)s->accel_rejected,
      (long)s->quaternion[0],(long)s->quaternion[1],(long)s->quaternion[2],(long)s->quaternion[3],
      (long)s->bmp_centi_c,(unsigned long)s->pressure_pa,(long)s->altitude_mm,(long)s->voltage_mv,(long)s->current_ma10,
      (long)s->power_mw,(long)s->battery_percent,(long)s->aht_milli_c,(unsigned long)s->humidity_milli,
      s->sv,(unsigned long)s->dt_us,s->stationary,(unsigned long)s->mag_age_ms,s->mag_quality,s->fusion,
      (unsigned long)s->sensor_error,s->speed_mode?"WHEEL_SPEED_TEST":"ATTITUDE");
    /* Append diagnostics inside the same complete line, not as debug text. */
    if(n>1 && n<(int)sizeof(line))n= n-2 + snprintf(line+n-2,sizeof(line)-(size_t)(n-2),
        " MAG_RATIO_m=%ld MAG_INNOV_mdeg=%ld BIAS_mdps=%ld,%ld,%ld ENV_AGE_ms=%lu,%lu,%lu MAG_STATE=%s MAG_REASON=%s MAG_SAMPLE_AGE_ms=%lu MAG_IO_ERR=%lu ATT_MODE=%s CAL_SOURCE=%s CAL_MEASURED=%lu CAL_SAVE=%u\r\n",
        (long)s->mag_ratio_milli,(long)s->mag_innovation_mdeg,V(gyro_bias),U(env_age_ms),
        mag_states[s->mag_state<5U?s->mag_state:4U],mag_reasons[s->mag_reason<8U?s->mag_reason:7U],
        (unsigned long)s->mag_sample_age_ms,(unsigned long)s->mag_io_errors,
        s->mag_state==1U?"9DOF":"GYRO_ACCEL",s->calibration_source?"FLASH":"DEFAULTS",
        (unsigned long)s->calibration_measured,s->calibration_save_result);
    send(n,1);
  }
  if((uint32_t)(s->uptime_ms-diag_tick)>=1000U) {
    diag_tick=s->uptime_ms;
    TelemetryUartStats u=TelemetryUART_Stats();CommandStats c=CommandParser_Stats();
    int n=snprintf(line,sizeof(line),
      "DIAG=1 T_ms=%lu EC3=%ld,%ld,%ld EA3_mdeg=%ld,%ld,%ld ERJ3=%lu,%lu,%lu ERR_mdeg=%ld,%ld,%ld PIDP3_m=%ld,%ld,%ld PIDI3_m=%ld,%ld,%ld PIDD3_m=%ld,%ld,%ld ATK_u3=11667,5000,8000 ERPM_SIGNED_m=%ld,%ld,%ld ERPM_SP_m=%ld,%ld,%ld ERPM_ERR_m=%ld,%ld,%ld EV3=%u,%u,%u EF3=%u,%u,%u ELIM3=%u,%u,%u RPM_LIMIT_m=%ld,%ld,%ld WREADY3=%u,%u,%u WKP_u6=%ld,%ld,%ld WKI_u6=%ld,%ld,%ld WKD_u6=%ld,%ld,%ld TX_DROP=%lu RX_DROP=%lu UART_ERR=%lu CMD_BAD=%lu CMD_DROP=%lu CTRL_DT_us=%lu CTRL_LATE=%lu\r\n",
      (unsigned long)s->uptime_ms,V(count),V(encoder_angle),U(encoder_rejected),V(error),V(p_term),V(i_term),V(d_term),
      V(rpm_signed),V(rpm_target),V(rpm_error),B(encoder_valid),B(encoder_fault),B(encoder_limit),V(rpm_limit),B(wheel_ready),
      V(wheel_kp),V(wheel_ki),V(wheel_kd),(unsigned long)u.tx_dropped,(unsigned long)u.rx_dropped,(unsigned long)u.uart_errors,
      (unsigned long)c.malformed,(unsigned long)c.queue_dropped,(unsigned long)s->control_dt_us,(unsigned long)s->control_late);
    if(n>1 && n<(int)sizeof(line))n= n-2 + snprintf(line+n-2,sizeof(line)-(size_t)(n-2),
        " ERPM_RAW_m=%ld,%ld,%ld EAGE_ms=%lu,%lu,%lu EPIN3=%u,%u,%u PPR=25 DIR_VALID=0 ERROR_FRAME=BODY_ROTVEC ENC_INSTALLED3=%u,%u,%u\r\n",
        V(raw_rpm),U(encoder_age),B(encoder_pin),B(encoder_installed));
    if(n>1 && n<(int)sizeof(line))n=n-2+snprintf(line+n-2,sizeof(line)-(size_t)(n-2),
        " EREC3=%lu,%lu,%lu ENC_NO_PULSE_STOP=0 GYRO_FS_dps=1000 GYRO_CLIP=%lu ACC_CLIP=%lu ACC_R_u6=%lu\r\n",U(encoder_recovery),
        (unsigned long)s->gyro_clip_count,(unsigned long)s->accel_clip_count,(unsigned long)s->accel_noise_u6);
    send(n,2);
  }
}
uint8_t TelemetryProtocol_Event(const uint8_t *data,size_t length) {
  if(!data||length<2)return 1;
  /* Only command/calibration records enter the radio. Boot banners, I2C scans
     and free-text diagnostics stay off the transparent telemetry stream. */
  if((length>=4&&!memcmp(data,"ACK ",4)) || (length>=5&&!memcmp(data,"NACK ",5))
      || (length>=4&&!memcmp(data,"CAL ",4)) || (length>=7&&!memcmp(data,"MAG CAL",7)))
    return TelemetryUART_Send(data,length,1,0)?0:1;
  if(length>=14&&!memcmp(data,"9DOF EKF READY",14)) {
    const char ready[]="CAL STATUS TYPE=FUSION PHASE=READY\r\n";
    return TelemetryUART_Send(ready,sizeof(ready)-1U,1,0)?0:1;
  }
  return 0;
}
