static void test_geometry(void){
  float q[4]={1,0,0,0},t[3]={0,0,90},e[3];
  assert(ADCS_AttitudeError(q,t,e));assert(fabsf(e[2]-90)<.001f);
  q[0]=-1;assert(ADCS_AttitudeError(q,t,e));assert(fabsf(e[2]-90)<.001f);
  float h=179*DEG_TO_RAD*.5f;q[0]=cosf(h);q[3]=sinf(h);t[2]=-179;
  assert(ADCS_AttitudeError(q,t,e));assert(fabsf(e[2]-2)<.001f);
  /* At roll=90, a world Z heading change is a body +Y rotation. */
  q[0]=q[1]=sqrtf(.5f);q[2]=q[3]=0;t[0]=90;t[1]=0;t[2]=10;
  assert(ADCS_AttitudeError(q,t,e));assert(fabsf(e[0])<.001f&&fabsf(e[1]-10)<.001f&&fabsf(e[2])<.001f);
  q[0]=NAN;assert(!ADCS_AttitudeError(q,t,e));
  fusion_status=HAL_OK;pid_enabled[2]=1;QuaternionNormalize(q);
  assert(fusion_status==HAL_ERROR && pid_enabled[2] && isfinite(q[0]));
  puts("PASS quaternion shortest path, q/-q, mixed axes, invalid quaternion");
}
static void test_pid(void){
  AxisControl_t c={0};
  for(int i=0;i<10000;i++)assert(AxisControl_Update(&c,0,90,0,28,1.2f,4,80,-1,.005f,1)==-255);
  assert(c.integral==0);assert(c.saturated);
  AxisControl_Update(&c,0,0,0,28,1.2f,4,80,-1,.005f,0);
  assert(c.output_pwm==0 && c.i_term==0);
  int command=AxisControl_Update(&c,0,1,0,28,1.2f,4,80,1,.005f,1);
  assert(command==28 && c.error_deg==1);
  puts("PASS actual PID output clamp, saturation anti-windup, disabled reset");
}
static void test_encoder(void){
  memset(wheel_encoder,0,sizeof(wheel_encoder));TIM_TypeDef timer={0};
  WheelEncoder_t *enc=&wheel_encoder[2];motor_command[2]=100;
  WheelEncoder_UpdateOne(enc,&timer,1500);assert(enc->no_pulse_fault==1);
  timer.CNT=25;WheelEncoder_UpdateOne(enc,&timer,1550);
  assert(enc->no_pulse_fault==1 && enc->pulse_seen && enc->last_edge_tick_ms==1550);
  assert(fabsf(enc->raw_rpm-1200)<.001f);
  Motor_ApplySupervisedCommand(2,120);assert(motor_command[2]==0);
  /* A pulse never self-clears a fault. Noise cannot bypass raw overspeed. */
  enc->no_pulse_fault=0;enc->rpm=100;enc->raw_rpm=4900;enc->speed_valid=1;
  Motor_ApplySupervisedCommand(2,120);assert(motor_command[2]==0&&enc->overspeed_limited);
  enc->raw_rpm=4700;Motor_ApplySupervisedCommand(2,120);assert(motor_command[2]==0);
  enc->raw_rpm=4500;Motor_ApplySupervisedCommand(2,120);assert(motor_command[2]==120);
  timer.CNT+=100000;WheelEncoder_UpdateOne(enc,&timer,1600);
  assert((enc->no_pulse_fault&2)&&!enc->speed_valid);
  memset(enc,0,sizeof(*enc));enc->counter_is_16_bit=1;enc->previous_count=65530;
  enc->count=INT32_MAX-2;timer.CNT=4;WheelEncoder_UpdateOne(enc,&timer,50);
  assert(enc->last_delta_count==10 && enc->count==INT32_MAX);
  assert(fabsf(enc->raw_rpm-480)<.001f);
  puts("PASS actual encoder PPR, timer wrap, fault latch, raw overspeed/hysteresis, noise, total saturation");
}
static void test_accel_covariance(void){
  float bias[3]={0},accel[3]={0,0,16384};AttitudeEKF_Init(bias,accel);
  attitude_ekf.covariance[0][3]=attitude_ekf.covariance[3][0]=.0001f;
  float before=attitude_ekf.covariance[3][3];
  for(int i=0;i<5000;i++)AttitudeEKF_AccelerometerUpdate(0,0,16384);
  assert(fabsf(attitude_ekf.covariance[3][3]-before)<1e-9f);
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++){
    assert(isfinite(attitude_ekf.covariance[i][j]));
    assert(fabsf(attitude_ekf.covariance[i][j]-attitude_ekf.covariance[j][i])<1e-7f);
  }
  puts("PASS actual accel update: no false bias covariance reduction, finite symmetric covariance");
}
static void test_fusion(void){
  float bias[3]={0},accel[3]={0,0,16384};AttitudeEKF_Init(bias,accel);
  memset(motor_command,0,sizeof(motor_command));memset(wheel_encoder,0,sizeof(wheel_encoder));
  memset(&magnetometer,0,sizeof(magnetometer));magnetometer.reference_valid=1;
  magnetometer.reference_world[0]=.8f;magnetometer.reference_world[2]=.6f;magnetometer.reference_norm=1;
  float truth=0;
  int16_t ax=(int16_t)lroundf(accel_offset_raw[0]),ay=(int16_t)lroundf(accel_offset_raw[1]);
  int16_t az=(int16_t)lroundf(accel_offset_raw[2]-16384/accel_scale[2]);
  for(int n=0;n<24000;n++){
    float rate=(n>=8000 && n<8400)?45:0;truth+=rate*.005f;
    int16_t gz=(int16_t)lroundf(-(rate+.573f)*131);
    tick=(uint32_t)(n+1)*5;
    AttitudeEKF_Update(ax,ay,az,0,0,gz,.005f);
    if(n%4==0){float mag[3]={.8f*cosf(truth*DEG_TO_RAD),-.8f*sinf(truth*DEG_TO_RAD),.6f};AttitudeEKF_MagnetometerUpdate(mag);}
  }
  float roll,pitch,yaw;QuaternionToEuler(attitude_ekf.q,&roll,&pitch,&yaw);
  printf("Actual ESKF synthetic 120s: truth=%.2f yaw=%.4f, biasZ=%.4f deg/s; accepted=%lu rejected=%lu\n",truth,yaw,attitude_ekf.gyro_bias_rad_s[2]*RAD_TO_DEG,(unsigned long)attitude_ekf.mag_accepted,(unsigned long)attitude_ekf.mag_rejected);
  assert(fabsf(yaw-truth)<.1f && fabsf(roll)<.1f && fabsf(pitch)<.1f);
  /* Failed incomplete mag capture retains previous coefficients. */
  magnetometer.offset[0]=123;magnetometer.offset[1]=-234;magnetometer.offset[2]=345;
  assert(GY511_MagCalibrate()==HAL_ERROR);
  assert(magnetometer.offset[0]==123 && magnetometer.offset[1]==-234 && magnetometer.offset[2]==345);
  puts("PASS actual gyro propagation+accel+mag retains rotated 90deg pose; failed mag capture preserves calibration");
}
static void test_compass_dropout(void){
  float bias[3]={0},accel[3]={0,0,16384};AttitudeEKF_Init(bias,accel);
  attitude_ekf.q[0]=cosf(22.5f*DEG_TO_RAD);attitude_ekf.q[3]=sinf(22.5f*DEG_TO_RAD);
  AttitudeEKF_RefreshEuler();
  memset(&magnetometer,0,sizeof(magnetometer));magnetometer.reference_valid=1;
  magnetometer.reference_norm=1000;magnetometer.reference_world[0]=.8f;magnetometer.reference_world[2]=.6f;
  for(unsigned i=0;i<3;i++)magnetometer.scale[i]=1;
  fusion_status=HAL_OK;mag_status=HAL_OK;pid_enabled[2]=1;
  float initial_q[4];memcpy(initial_q,attitude_ekf.q,sizeof(initial_q));
  for(unsigned i=0;i<20;i++){tick+=20;Magnetometer_ProcessSample(HAL_ERROR,0,0,0);}
  assert(mag_status==HAL_ERROR && fusion_status==HAL_OK && pid_enabled[2]);
  assert(!memcmp(initial_q,attitude_ekf.q,sizeof(initial_q)));
  int16_t ax=(int16_t)lroundf(accel_offset_raw[0]),ay=(int16_t)lroundf(accel_offset_raw[1]);
  int16_t az=(int16_t)lroundf(accel_offset_raw[2]-16384/accel_scale[2]);
  AxisControl_t control={0};control.setpoint_deg=45;
  for(unsigned i=0;i<1200;i++){
    tick+=5;AttitudeEKF_Update(ax,ay,az,0,0,0,.005f);
    if(i%4==0)Magnetometer_ProcessSample(HAL_ERROR,0,0,0);
    float target[3]={0,0,45},error[3];assert(ADCS_AttitudeError(attitude_ekf.q,target,error));
    (void)AxisControl_Update(&control,attitude_ekf.yaw_deg,error[2],0,28,1.2f,4,80,-1,.005f,pid_enabled[2]);
  }
  assert(pid_enabled[2] && fusion_status==HAL_OK && fabsf(attitude_ekf.yaw_deg-45)<.1f);
  Magnetometer_ProcessSample(HAL_OK,10000,0,0);
  assert(mag_reject_reason==1 && pid_enabled[2] && fusion_status==HAL_OK);
  Magnetometer_ProcessSample(HAL_OK,-566,566,600);
  assert(mag_reject_reason==3 && fabsf(mag_innovation_deg)>100 && pid_enabled[2]);
  Magnetometer_ProcessSample(HAL_OK,566,-566,600);
  assert(mag_status==HAL_OK && mag_reject_reason==0 && pid_enabled[2] && fusion_status==HAL_OK);
  assert(fabsf(attitude_ekf.yaw_deg-45)<.1f && control.setpoint_deg==45);
  puts("PASS actual 6s compass I2C dropout, rejected norm/heading and recovery: PID stays enabled, IMU runs, angle/target retained");
}
static void test_operator_authority(void){
  control_targets_initialized=1;pid_enabled[0]=1;pid_enabled[1]=0;pid_enabled[2]=1;
  roll_control.setpoint_deg=10;pitch_control.setpoint_deg=20;yaw_control.setpoint_deg=90;
  attitude_ekf.yaw_deg=45;attitude_ekf.roll_deg=0;attitude_ekf.pitch_deg=0;
  AttitudeControl_Reset();
  assert(pid_enabled[0]&&!pid_enabled[1]&&pid_enabled[2]);
  assert(roll_control.setpoint_deg==10&&pitch_control.setpoint_deg==20&&yaw_control.setpoint_deg==90);
  float bias[3]={0},accel[3]={0,0,16384};AttitudeEKF_Init(bias,accel);
  assert(fabsf(attitude_ekf.yaw_deg-45)<.001f);
  AttitudeControl_Reset();assert(pid_enabled[2]&&yaw_control.setpoint_deg==90);
  float measured_before[4];memcpy(measured_before,attitude_ekf.q,sizeof(measured_before));
  AttitudeControl_HoldCurrentYaw();
  assert(fabsf(yaw_control.setpoint_deg-45)<.001f && pid_enabled[2]);
  assert(!memcmp(measured_before,attitude_ekf.q,sizeof(measured_before)));
  puts("PASS operator PID latch/targets survive controller and EKF recovery; retained yaw is measured 45deg, not target 90deg");
  control_timing_paused=0;
  assert(ControlTiming_Accept(5,0)&&!control_timing_paused);
  assert(ControlTiming_Accept(25,100)&&control_timing_paused);
  assert(ControlTiming_Accept(5,199)&&control_timing_paused);
  assert(ControlTiming_Accept(5,200)&&!control_timing_paused);
  assert(!ControlTiming_Accept(101,250)&&control_timing_paused&&pid_enabled[2]);
  control_timing_paused=0;
  assert(ControlTiming_Accept(25,0xFFFFFFF0U)&&control_timing_paused);
  assert(ControlTiming_Accept(5,83)&&control_timing_paused);
  assert(ControlTiming_Accept(5,84)&&!control_timing_paused&&pid_enabled[2]);
  puts("PASS timing pause/recovery and tick wrap: 25ms does not reinitialize EKF, >100ms requests recovery but never clears operator PID");
}
int main(void){test_geometry();test_pid();test_encoder();test_accel_covariance();test_fusion();test_compass_dropout();test_operator_authority();}
