#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H
#include <stdint.h>
typedef struct {
  uint32_t uptime_ms,sequence,mag_age_ms,mag_rejected,mag_accepted,accel_accepted,accel_rejected;
  uint32_t pressure_pa,humidity_milli,dt_us,control_dt_us,control_late,sensor_error;
  uint32_t imu_age_ms,encoder_age[3];
  uint32_t encoder_recovery[3];
  uint32_t gyro_clip_count,accel_clip_count,accel_noise_u6;
  uint32_t env_age_ms[3];
  uint32_t mag_sample_age_ms,mag_io_errors,calibration_measured;
  uint8_t mag_state,mag_reason,calibration_source,calibration_save_result;
  int32_t body_error[3],gyro_bias[3],raw_rpm[3],mag_ratio_milli,mag_innovation_mdeg;
  uint8_t encoder_pin[3],pid_active[3],pid_block[3],encoder_installed[3];
  int32_t angle[3],target[3],error[3],gyro[3],accel[3],mag[3],quaternion[4],yaw_rate;
  int32_t pid_output[3],requested_pwm[3],pwm[3],rpm_milli[3],count[3],encoder_angle[3];
  int32_t p_term[3],i_term[3],d_term[3],rpm_signed[3],rpm_target[3],rpm_error[3],rpm_limit[3];
  int32_t wheel_kp[3],wheel_ki[3],wheel_kd[3];
  uint32_t encoder_rejected[3];
  int32_t bmp_centi_c,altitude_mm,voltage_mv,current_ma10,power_mw,battery_percent,aht_milli_c;
  uint8_t who,sv,pid[3],estop,charging,stationary,mag_quality,attitude_valid,fusion;
  uint8_t encoder_valid[3],encoder_fault[3],encoder_limit[3],wheel_ready[3],speed_mode;
} TelemetryData;
#endif
