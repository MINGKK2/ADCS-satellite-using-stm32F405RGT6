/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "heading_update.h"
#include "telemetry_uart.h"
#include "telemetry_protocol.h"
#include "command_parser.h"
#include "usart.h"
#include "dma.h"
#include "calibration_store.h"
#include "axis_frame.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void App_Wait(uint32_t milliseconds);
static void App_CommunicationService(uint8_t busy);
static void App_BuildSnapshot(TelemetryData *data);
static void Motors_StopAll(void);
static void Calibration_Load(void);
static void Calibration_Save(uint32_t measured_flags);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MPU6050_ADDR_LOW      (0x68U << 1)
#define MPU6050_ADDR_HIGH     (0x69U << 1)

#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6500_ACCEL_CONFIG2 0x1D
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75

/* GY-511 boards exist with either LSM303DLHC or LSM303D. */
#define LSM303DLHC_MAG_ADDR       (0x1EU << 1)
#define LSM303DLHC_CRA_REG        0x00
#define LSM303DLHC_CRB_REG        0x01
#define LSM303DLHC_MR_REG         0x02
#define LSM303DLHC_OUT_X_H        0x03
#define LSM303DLHC_STATUS_REG     0x09

#define LSM303D_ADDR_LOW          (0x1DU << 1)
#define LSM303D_ADDR_HIGH         (0x1EU << 1)
#define LSM303D_WHO_AM_I_REG      0x0F
#define LSM303D_WHO_AM_I_VALUE    0x49
#define LSM303D_STATUS_M_REG      0x07
#define LSM303D_OUT_X_L_M         0x08
#define LSM303D_CTRL5             0x24
#define LSM303D_CTRL6             0x25
#define LSM303D_CTRL7             0x26

/* Optional slow sensors sharing I2C1 with MPU6050 and GY-511. */
#define BMP280_ADDR_LOW           (0x76U << 1)
#define BMP280_ADDR_HIGH          (0x77U << 1)
#define BMP280_CHIP_ID_REG        0xD0
#define BMP280_CHIP_ID_VALUE      0x58
#define BMP280_RESET_REG          0xE0
#define BMP280_CALIB_REG          0x88
#define BMP280_PRESS_MSB_REG      0xF7
#define BMP280_CTRL_MEAS_REG      0xF4
#define BMP280_CONFIG_REG         0xF5

#define INA219_ADDR               (0x40U << 1)
#define INA219_CONFIG_REG         0x00
#define INA219_SHUNT_REG          0x01
#define INA219_BUS_REG            0x02
/* Most INA219 breakout boards use a 0.1-ohm (100-milliohm) shunt. */
#define INA219_SHUNT_MILLIOHMS    100L

#define AHT30_ADDR                (0x38U << 1)
#define AHT30_MEASUREMENT_TIME_MS 85U

#define CONTROL_PERIOD_MS     5U
#define ENCODER_CONTROL_PERIOD_MS 50U
#define MAG_UPDATE_PERIOD_MS  20U
#define MAG_CALIBRATION_TIME_MS 20000U
#define BMP280_UPDATE_PERIOD_MS  200U
#define INA219_UPDATE_PERIOD_MS  100U
#define AHT30_UPDATE_PERIOD_MS  2000U
#define OPTIONAL_I2C_TIMEOUT_MS 3U

#define MOTOR_PWM_TOP         255U
#define MOTOR_TIMER_PSC       15U
#define MOTOR_MAX_PWM         255
#define MOTOR_MIN_PWM         0
#define MOTOR_COMMAND_DEADBAND 0
/* Motor command polarity is independent of the sensor-to-body axis mapping. */
#define ROLL_MOTOR_CONTROL_SIGN   (1.0f)
#define PITCH_MOTOR_CONTROL_SIGN  (1.0f)
#define YAW_MOTOR_CONTROL_SIGN   (-1.0f)

/* Existing outer-loop attitude gains retained as the current baseline.
   They are not claimed as plant-identified values; verify them from logged
   step tests before treating the tuning as final. */
#define ROLL_KP               (28.0f / 2.4f)
#define ROLL_KI                5.0f
#define ROLL_KD                8.0f
#define ROLL_INTEGRAL_LIMIT    80.0f
#define PITCH_KP              (28.0f / 2.4f)
#define PITCH_KI               5.0f
#define PITCH_KD               8.0f
#define PITCH_INTEGRAL_LIMIT   80.0f
#define YAW_KP                (28.0f / 2.4f)
#define YAW_KI                5.0f
#define YAW_KD                8.0f
#define YAW_INTEGRAL_LIMIT     80.0f

#define DEG_TO_RAD            0.017453292519943295f
#define RAD_TO_DEG            57.29577951308232f
#define GYRO_LSB_PER_DPS       32.8f
#define ACCEL_LSB_PER_G        16384.0f

/* Six-face accelerometer calibration measured on this module. */
#define ACCEL_OFFSET_X_RAW_DEFAULT      229.7750f
#define ACCEL_OFFSET_Y_RAW_DEFAULT      393.3750f
#define ACCEL_OFFSET_Z_RAW_DEFAULT     (-853.8150f)
#define ACCEL_SCALE_X_DEFAULT           1.0550736f
#define ACCEL_SCALE_Y_DEFAULT           1.0512588f
#define ACCEL_SCALE_Z_DEFAULT           1.0254462f

#define EKF_GYRO_NOISE_RAD_S   0.020f
#define EKF_BIAS_WALK_RAD_S2   0.0005f
#define EKF_ACCEL_NOISE        0.040f
/* Do not let acceleration or motor magnetic fields look like attitude. */
#define ACCEL_GATE_MIN_G       0.94f
#define ACCEL_GATE_MAX_G       1.06f
#define ACCEL_INNOVATION_MIN_DOT 0.90f
#define ACCEL_DYNAMIC_RESIDUAL_G 0.030f
#define ACCEL_MAX_CORRECTION_RAD 0.00872664626f
#define MAG_FIELD_MIN_RATIO    0.72f
#define MAG_FIELD_MAX_RATIO    1.28f
#define MAG_INNOVATION_MIN_DOT 0.82f
#define STATIONARY_ACCEL_TOL_G 0.025f
#define STATIONARY_GYRO_DPS    0.80f
#define STATIONARY_CONFIRM_MS  800U
#define GYRO_CAL_MAX_RANGE_RAW ((int32_t)(6.1f*GYRO_LSB_PER_DPS))

/* Last known-good calibration for this GY-511. A manual CAL MAG command
   replaces these values in RAM until the next reset. */
#define MAG_OFFSET_X_DEFAULT   (-643.5f)
#define MAG_OFFSET_Y_DEFAULT   (-493.0f)
#define MAG_OFFSET_Z_DEFAULT     587.5f
#define MAG_SCALE_X_DEFAULT       0.969061f
#define MAG_SCALE_Y_DEFAULT       1.043096f
#define MAG_SCALE_Z_DEFAULT       0.990697f

/* Reaction-wheel axis identified from a bidirectional rotation test. */
#define YAW_LEGACY_AXIS_X      (-0.0999073f)
#define YAW_LEGACY_AXIS_Y        0.0832897f
#define YAW_LEGACY_AXIS_Z        0.9915046f
#define YAW_AXIS_X             YAW_LEGACY_AXIS_Y
#define YAW_AXIS_Y           (-YAW_LEGACY_AXIS_X)
#define YAW_AXIS_Z             YAW_LEGACY_AXIS_Z

/* Each wheel uses one digital optical output.  With 25 slots and rising-edge
   counting there are exactly 25 pulses/revolution.  A one-channel encoder
   reports speed magnitude only; rotation direction is not observable. */
#define WHEEL_ENCODER_PULSES_PER_REV  25.0f
#define WHEEL_ENCODER_FILTER_ALPHA    0.35f
#define WHEEL_ENCODER_MAX_RPM         30000.0f
#define ENCODER_CLEAR_VERIFY_PULSES   25U

/* GY-511 axes to body axes. This default assumes the module X/Y/Z arrows are
   mounted parallel to body X/Y/Z. Change signs/order here if the board is
   mounted differently. */
#define MAG_BODY_X_SIGN        (1.0f)
#define MAG_BODY_Y_SIGN        (1.0f)
#define MAG_BODY_Z_SIGN        (1.0f)
#define MAG_BODY_X_SENSOR_AXIS 0U
#define MAG_BODY_Y_SENSOR_AXIS 1U
#define MAG_BODY_Z_SENSOR_AXIS 2U

static uint16_t mpu6050_address = MPU6050_ADDR_LOW;

typedef struct
{
  float q[4];
  float gyro_bias_rad_s[3];
  float covariance[6][6];
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float previous_yaw_deg;
  float yaw_rate_dps;
  uint32_t accel_accepted;
  uint32_t accel_rejected;
  uint32_t mag_accepted;
  uint32_t mag_rejected;
  uint32_t last_dt_us;
  uint32_t stationary_since_ms;
  uint8_t stationary;
  uint8_t accel_history_valid;
  float previous_accel_raw[3];
  float accel_observation_variance;
  uint32_t gyro_clipped_samples,accel_clipped_samples;
} AttitudeEkf_t;

typedef struct
{
  float angle_deg;
  float setpoint_deg;
  float error_deg;
  float integral;
  float rate_dps;
  float p_term;
  float i_term;
  float d_term;
  float output_pwm;
  uint8_t saturated;
} AxisControl_t;

typedef enum
{
  MOTOR_AXIS_ROLL = 0,
  MOTOR_AXIS_PITCH = 1,
  MOTOR_AXIS_YAW = 2,
  MOTOR_AXIS_COUNT = 3
} MotorAxis_t;

typedef struct
{
  uint32_t previous_count;
  uint32_t previous_tick_ms;
  uint32_t rejected_samples;
  int32_t count;
  float angle_deg;
  float rpm;
  float signed_rpm_estimate;
  int8_t assumed_direction;
  uint32_t last_delta_count;
  uint32_t last_edge_tick_ms;
  float raw_rpm;
  uint8_t pulse_seen;
  uint8_t fault_flags; /* EF3 0x02: implausible pulse rate; 0x01 is no longer generated. */
  uint8_t overspeed_limited;
  uint8_t counter_is_16_bit;
  uint8_t speed_valid;
  uint32_t recovery_pulses;
} WheelEncoder_t;

typedef struct
{
  float setpoint_rpm;
  float feedback_rpm;
  float error_rpm;
  float integral;
  float previous_feedback_rpm;
  float kp;
  float ki;
  float kd;
  float p_term;
  float i_term;
  float d_term;
  float output_pwm;
  uint8_t gains_configured;
  uint8_t saturated;
} WheelSpeedControl_t;

typedef struct
{
  float offset[3];
  float scale[3];
  float body[3];
  float reference_world[3];
  float reference_norm;
  uint8_t calibrated;
  uint8_t reference_valid;
  uint32_t rejected_samples;
} Magnetometer_t;

typedef struct
{
  float face_mean[3][2][3];
  uint8_t captured[3][2];
  uint8_t count;
  uint8_t active;
} AccelSixPointCalibration_t;

typedef struct
{
  uint16_t address;
  uint8_t chip_id;
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;
  uint16_t dig_p1;
  int16_t dig_p2;
  int16_t dig_p3;
  int16_t dig_p4;
  int16_t dig_p5;
  int16_t dig_p6;
  int16_t dig_p7;
  int16_t dig_p8;
  int16_t dig_p9;
  int32_t t_fine;
  int32_t temperature_centi_c;
  uint32_t pressure_pa;
  uint32_t failures;
  uint8_t valid;
} Bmp280_t;

typedef struct
{
  int32_t bus_mv;
  int32_t shunt_uv;
  int32_t current_ma_x10;
  int32_t power_mw;
  uint32_t failures;
  uint8_t valid;
} Ina219_t;

typedef struct
{
  int32_t temperature_milli_c;
  uint32_t humidity_milli_percent;
  uint32_t trigger_tick;
  uint32_t failures;
  uint8_t measurement_pending;
  uint8_t valid;
} Aht30_t;



typedef enum
{
  SYSTEM_MODE_WORKING = 0,
  SYSTEM_MODE_CHARGING = 1
} SystemMode_t;

typedef enum
{
  CONTROL_MODE_ATTITUDE = 0,
  CONTROL_MODE_WHEEL_SPEED_TEST = 1
} ControlMode_t;

typedef enum
{
  GY511_MAG_NONE = 0,
  GY511_MAG_LSM303DLHC = 1,
  GY511_MAG_LSM303D = 2
} Gy511MagType_t;

static AttitudeEkf_t attitude_ekf = {0};
static AxisControl_t roll_control = {0};
static AxisControl_t pitch_control = {0};
static AxisControl_t yaw_control = {0};
static WheelEncoder_t wheel_encoder[MOTOR_AXIS_COUNT] = {0};
static WheelSpeedControl_t wheel_speed_control[MOTOR_AXIS_COUNT] = {0};
static Magnetometer_t magnetometer =
{
  .offset = {MAG_OFFSET_X_DEFAULT, MAG_OFFSET_Y_DEFAULT, MAG_OFFSET_Z_DEFAULT},
  .scale = {MAG_SCALE_X_DEFAULT, MAG_SCALE_Y_DEFAULT, MAG_SCALE_Z_DEFAULT},
  .calibrated = 1U
};
static AccelSixPointCalibration_t accel_calibration = {0};
static Gy511MagType_t gy511_mag_type = GY511_MAG_NONE;
static uint16_t gy511_mag_address = 0U;
static uint8_t gy511_mag_who_am_i = 0U;
static Bmp280_t bmp280 = {0};
static Ina219_t ina219 = {0};
static Aht30_t aht30 = {0};
static HAL_StatusTypeDef bmp280_status = HAL_ERROR;
static HAL_StatusTypeDef ina219_status = HAL_ERROR;
static HAL_StatusTypeDef aht30_status = HAL_ERROR;
static uint32_t last_bmp280_tick = 0U;
static uint32_t last_ina219_tick = 0U;
static uint32_t last_aht30_tick = 0U;
static uint32_t last_env_sample_ms[3]={0};
static int32_t motor_command[MOTOR_AXIS_COUNT] = {0, 0, 0};
static int32_t requested_motor_command[MOTOR_AXIS_COUNT] = {0, 0, 0};
static float wheel_rpm_limit[MOTOR_AXIS_COUNT] = {4800.0f, 4800.0f, 4800.0f};
static float pending_setpoint_deg[MOTOR_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
static float pending_wheel_rpm[MOTOR_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
static float pending_wheel_pid_gain[3] = {0.0f, 0.0f, 0.0f};
static float pending_rpm_limit[MOTOR_AXIS_COUNT] = {0.0f, 0.0f, 0.0f};
/* Explicit arm after boot; loss of the ground station never disarms PID. */
/* Operator request latch. Only PID commands/explicit ESTOP may change it. */
static uint8_t pid_enabled[MOTOR_AXIS_COUNT] = {0U, 0U, 0U};
static uint8_t control_targets_initialized=0U;
static uint32_t control_pause_until_ms=0U;
static uint8_t control_timing_paused=0U;
static uint8_t pending_pid_axis = MOTOR_AXIS_COUNT;
static uint8_t pending_wheel_axis = MOTOR_AXIS_COUNT;
static uint8_t pending_pid_enable = 0U;
static volatile uint8_t emergency_stop = 0U;
static uint8_t motor_initialized=0U;
static uint8_t app_busy=0U;
static uint8_t mpu_who_am_i;
static HAL_StatusTypeDef mpu_status=HAL_ERROR,mag_status=HAL_ERROR,fusion_status=HAL_ERROR;
static int16_t last_imu_raw[6];
static uint32_t control_actual_dt_us,control_late;
static TelemetryData telemetry_data;
static SystemMode_t system_mode = SYSTEM_MODE_WORKING;
static ControlMode_t control_mode = CONTROL_MODE_ATTITUDE;
static uint32_t last_heading_update_ms=0U;
static uint32_t clean_heading_samples=0U;
static uint8_t magnetic_heading_quality=0U;
static float mag_field_ratio=0.0f,mag_innovation_deg=0.0f;
static uint32_t last_imu_sample_ms=0U;
static uint32_t last_mag_sample_ms=0U,mag_io_errors=0U;
static uint32_t mag_read_failures=0U;
/* Reject reason: 0=none, 1=field norm, 2=horizontal field, 3=innovation,
   4=filter, 5=overflow, 6=no fresh sample, 7=I2C/device. */
static uint8_t mag_reject_reason=6U;
static uint32_t mag_reference_epoch=0U;
static float calibration_gyro_bias[3]={0};
static uint8_t calibration_gyro_valid=0U,calibration_from_flash=0U;
static uint32_t calibration_measured_flags=0U;
static CalibrationStoreResult calibration_save_result=CAL_STORE_EMPTY;
static uint32_t pressure_reference_pa = 0U;
static int32_t relative_altitude_mm = 0;
static char last_received_command[COMMAND_LINE_SIZE] = {0};
static float accel_offset_raw[3] =
{
  ACCEL_OFFSET_X_RAW_DEFAULT,
  ACCEL_OFFSET_Y_RAW_DEFAULT,
  ACCEL_OFFSET_Z_RAW_DEFAULT
};
static float accel_scale[3] =
{
  ACCEL_SCALE_X_DEFAULT,
  ACCEL_SCALE_Y_DEFAULT,
  ACCEL_SCALE_Z_DEFAULT
};

static void Calibration_Load(void)
{
  CalibrationData data;
  if(CalibrationStore_Load(&data)!=CAL_STORE_OK || data.mpu_who!=mpu_who_am_i)return;
  if(data.flags&CALIBRATION_ACCEL){memcpy(accel_offset_raw,data.accel_offset,sizeof(accel_offset_raw));memcpy(accel_scale,data.accel_scale,sizeof(accel_scale));}
  if((data.flags&CALIBRATION_MAG) && (!gy511_mag_type || data.mag_type==(uint32_t)gy511_mag_type)){
    memcpy(magnetometer.offset,data.mag_offset,sizeof(magnetometer.offset));
    memcpy(magnetometer.scale,data.mag_scale,sizeof(magnetometer.scale));magnetometer.calibrated=1U;
  }
  if(data.flags&CALIBRATION_GYRO){ADCS_GyroBiasLoad(data.gyro_bias,calibration_gyro_bias);calibration_gyro_valid=1U;}
  calibration_measured_flags=data.measured_flags;calibration_from_flash=1U;calibration_save_result=CAL_STORE_OK;
}
static void Calibration_Save(uint32_t measured_flags)
{
  calibration_measured_flags |= measured_flags;
  /* Never program internal flash while a wheel is enabled. */
  /* A retained ON request is not an energized motor. Calibration routines
     suspend all motor outputs before saving, without changing that request. */
  for(unsigned i=0;i<3;i++)if(motor_command[i] || requested_motor_command[i])return;
  CalibrationData data;memset(&data,0,sizeof(data));data.layout_tag=CALIBRATION_LAYOUT_TAG;
  data.flags=CALIBRATION_ACCEL|CALIBRATION_MAG|(calibration_gyro_valid?CALIBRATION_GYRO:0U);
  data.measured_flags=calibration_measured_flags;data.mpu_who=mpu_who_am_i;data.mag_type=(uint32_t)gy511_mag_type;
  memcpy(data.accel_offset,accel_offset_raw,sizeof(accel_offset_raw));memcpy(data.accel_scale,accel_scale,sizeof(accel_scale));
  memcpy(data.mag_offset,magnetometer.offset,sizeof(magnetometer.offset));memcpy(data.mag_scale,magnetometer.scale,sizeof(magnetometer.scale));
  ADCS_GyroBiasSave(calibration_gyro_bias,data.gyro_bias);
  calibration_save_result=CalibrationStore_Save(&data);
  if(calibration_save_result==CAL_STORE_OK)calibration_from_flash=1U;
  char message[112];int n=snprintf(message,sizeof(message),"CAL %s TYPE=STORAGE RESULT=%u SOURCE=%s MEASURED=%lu\r\n",
      calibration_save_result==CAL_STORE_OK?"DONE":"WARN",calibration_save_result,calibration_from_flash?"FLASH":"RAM",
      (unsigned long)calibration_measured_flags);
  TelemetryProtocol_Event((const uint8_t *)message,(size_t)n);
}

static uint8_t ControlTiming_Accept(uint32_t elapsed_ms,uint32_t now)
{
  if(control_timing_paused && (int32_t)(now-control_pause_until_ms)>=0)control_timing_paused=0U;
  if(elapsed_ms>20U){control_pause_until_ms=now+100U;control_timing_paused=1U;}
  return elapsed_ms<=100U;
}

static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static float WrapAngle180(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  while (angle < -180.0f)
  {
    angle += 360.0f;
  }
  return angle;
}

static void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* Three synchronized L298N enable inputs on TIM3:
     PA6=CH1/ROLL ENA, PA7=CH2/PITCH ENA, PB0=CH3/YAW ENA. */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Direction inputs:
     ROLL  PC4=IN1 PC5=IN2
     PITCH PB12=IN1 PB13=IN2
     YAW   PB1=IN1 PB8=IN2 (PB2/BOOT1 remains unused). */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  HAL_GPIO_WritePin(
      GPIOB,
      GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_12 | GPIO_PIN_13,
      GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  TIM3->CR1 = 0;
  /* 84 MHz / (15 + 1) / 256 = 20.5 kHz; every CCR is 0..255. */
  TIM3->PSC = MOTOR_TIMER_PSC;
  TIM3->ARR = MOTOR_PWM_TOP;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  /* Direct CCR writes: stop/reversal must not wait for a preload update. */
  TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2
               | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  motor_initialized=1U;
}

static void Motor_SetAxisCommand(MotorAxis_t axis, int32_t command)
{
  GPIO_TypeDef *in1_port;
  GPIO_TypeDef *in2_port;
  uint16_t in1_pin;
  uint16_t in2_pin;
  volatile uint32_t *compare;

  switch (axis)
  {
    case MOTOR_AXIS_ROLL:
      in1_port = GPIOC;
      in1_pin = GPIO_PIN_4;
      in2_port = GPIOC;
      in2_pin = GPIO_PIN_5;
      compare = &TIM3->CCR1;
      break;

    case MOTOR_AXIS_PITCH:
      in1_port = GPIOB;
      in1_pin = GPIO_PIN_12;
      in2_port = GPIOB;
      in2_pin = GPIO_PIN_13;
      compare = &TIM3->CCR2;
      break;

    case MOTOR_AXIS_YAW:
    default:
      axis = MOTOR_AXIS_YAW;
      in1_port = GPIOB;
      in1_pin = GPIO_PIN_1;
      in2_port = GPIOB;
      in2_pin = GPIO_PIN_8;
      compare = &TIM3->CCR3;
      break;
  }

  if (command > MOTOR_MAX_PWM)
  {
    command = MOTOR_MAX_PWM;
  }
  else if (command < -MOTOR_MAX_PWM)
  {
    command = -MOTOR_MAX_PWM;
  }

  if ((command >= -MOTOR_COMMAND_DEADBAND) &&
      (command <= MOTOR_COMMAND_DEADBAND))
  {
    command = 0;
  }
  else if ((command > 0) && (command < MOTOR_MIN_PWM))
  {
    command = MOTOR_MIN_PWM;
  }
  else if ((command < 0) && (command > -MOTOR_MIN_PWM))
  {
    command = -MOTOR_MIN_PWM;
  }

  motor_command[axis] = command;

  uint32_t motor_irq_state=__get_PRIMASK();__disable_irq();
  if(emergency_stop){command=0;motor_command[axis]=0;}
  /* Disable ENA before changing direction. */
  uint32_t channel_enable=1UL << (4U*(uint32_t)axis);
  TIM3->CCER &= ~channel_enable;
  *compare = 0U;

  if (command > 0)
  {
    /* Forward: IN1=1, IN2=0, ENA receives PWM. */
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    *compare = (uint32_t)command;
  }
  else if (command < 0)
  {
    /* Reverse: IN1=0, IN2=1, ENA receives PWM. */
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    *compare = (uint32_t)(-command);
  }
  else
  {
    /* Coast: ENA=0 and both direction inputs low. */
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  TIM3->CCER |= channel_enable;
  __set_PRIMASK(motor_irq_state);
}

static int32_t Motor_ApplySupervisedCommand(
    MotorAxis_t axis,
    int32_t requested_command)
{
  int32_t applied_command = requested_command;
  requested_motor_command[axis] = requested_command;
  uint8_t previously_limited=wheel_encoder[axis].overspeed_limited;
  wheel_encoder[axis].overspeed_limited = 0U;

  if (wheel_encoder[axis].fault_flags != 0U)
  {
    applied_command = 0;
  }
  if (wheel_encoder[axis].speed_valid == 0U) applied_command = 0;
  if ((wheel_rpm_limit[axis] > 0.0f)
      && (wheel_encoder[axis].speed_valid != 0U)
      && (fmaxf(wheel_encoder[axis].raw_rpm,wheel_encoder[axis].rpm) >=
          wheel_rpm_limit[axis] * (previously_limited ? 0.95f : 1.0f)))
  {
    applied_command = 0;
    wheel_encoder[axis].overspeed_limited = 1U;
  }

  Motor_SetAxisCommand(axis, applied_command);
  return motor_command[axis];
}

static void Motors_StopAll(void)
{
  for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
  {
    requested_motor_command[axis] = 0;
  }
  Motor_SetAxisCommand(MOTOR_AXIS_ROLL, 0);
  Motor_SetAxisCommand(MOTOR_AXIS_PITCH, 0);
  Motor_SetAxisCommand(MOTOR_AXIS_YAW, 0);
}

static void WheelEncoder_ConfigTimer(TIM_TypeDef *timer, uint32_t period)
{
  timer->CR1 = 0;
  timer->SMCR = 0;
  timer->CCMR1 = 0;
  timer->CCER = 0;
  timer->PSC = 0;
  timer->ARR = period;

  /* TI1 is the external clock.  The maximum digital input filter suppresses
     short brush-motor/long-wire spikes before they reach the counter. */
  timer->CCMR1 = TIM_CCMR1_CC1S_0
               | TIM_CCMR1_IC1F_0
               | TIM_CCMR1_IC1F_1
               | TIM_CCMR1_IC1F_2
               | TIM_CCMR1_IC1F_3;
  timer->CCER = TIM_CCER_CC1E; /* Rising edge only. */
  timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2
              | TIM_SMCR_TS_0 | TIM_SMCR_TS_2; /* External clock mode 1, TI1FP1. */
  timer->CNT = 0;
  timer->EGR = TIM_EGR_UG;
  timer->SR = 0;
  timer->CR1 = TIM_CR1_CEN;
}

static void WheelEncoders_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t now = HAL_GetTick();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  /* One OUT wire per optical module:
     ROLL=PA8/TIM1_CH1, PITCH=PC6/TIM8_CH1, YAW=PA0/TIM2_CH1.
     Internal pull-ups are enabled; use an external 4.7 kOhm pull-up to 3.3 V
     when the module output is open collector or the wire is long. */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  WheelEncoder_ConfigTimer(TIM1, 0xFFFFU);
  WheelEncoder_ConfigTimer(TIM8, 0xFFFFU);
  WheelEncoder_ConfigTimer(TIM2, 0xFFFFFFFFU);

  for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
  {
    wheel_encoder[axis].previous_count = 0U;
    wheel_encoder[axis].previous_tick_ms = now;
    wheel_encoder[axis].rejected_samples = 0U;
    wheel_encoder[axis].count = 0;
    wheel_encoder[axis].angle_deg = 0.0f;
    wheel_encoder[axis].rpm = 0.0f;
    wheel_encoder[axis].signed_rpm_estimate = 0.0f;
    wheel_encoder[axis].assumed_direction = 0;
    wheel_encoder[axis].last_delta_count = 0U;
    wheel_encoder[axis].fault_flags = 0U;
    wheel_encoder[axis].overspeed_limited = 0U;
    wheel_encoder[axis].counter_is_16_bit =
        (axis == MOTOR_AXIS_YAW) ? 0U : 1U;
    wheel_encoder[axis].speed_valid = 0U;
    wheel_encoder[axis].recovery_pulses = 0U;
  }
}

static void WheelEncoder_UpdateOne(
    WheelEncoder_t *encoder,
    TIM_TypeDef *timer,
    uint32_t now_ms)
{
  uint32_t current_count = timer->CNT;
  uint32_t elapsed_ms = now_ms - encoder->previous_tick_ms;
  uint32_t axis = (uint32_t)(encoder - &wheel_encoder[0]);

  if (encoder->counter_is_16_bit != 0U)
  {
    current_count &= 0xFFFFU;
  }

  if (elapsed_ms > 0U)
  {
    uint32_t delta_count;
    float raw_rpm;

    if (encoder->counter_is_16_bit != 0U)
    {
      delta_count = (uint32_t)(uint16_t)(
          (uint16_t)current_count - (uint16_t)encoder->previous_count);
    }
    else
    {
      delta_count = current_count - encoder->previous_count;
    }

    raw_rpm = (float)delta_count * 60000.0f
            / (WHEEL_ENCODER_PULSES_PER_REV * (float)elapsed_ms);
    encoder->last_delta_count = delta_count;
    encoder->raw_rpm = raw_rpm;

    if (raw_rpm <= WHEEL_ENCODER_MAX_RPM)
    {
      if(delta_count <= (uint32_t)(INT32_MAX-encoder->count))encoder->count += (int32_t)delta_count;
      else encoder->count=INT32_MAX; /* diagnostic total saturates, never signed overflow */
      encoder->angle_deg = fmodf(
          encoder->angle_deg + (float)delta_count * 360.0f / WHEEL_ENCODER_PULSES_PER_REV,
          360.0f);

      if (encoder->speed_valid == 0U)
      {
        encoder->rpm = raw_rpm;
        encoder->speed_valid = 1U;
      }
      else
      {
        encoder->rpm += WHEEL_ENCODER_FILTER_ALPHA
                      * (raw_rpm - encoder->rpm);
      }

      if (delta_count > 0U)
      {
        /* Recovery is evidence collected while disarmed, never an automatic clear. */
        if(encoder->fault_flags && !pid_enabled[axis] && !motor_command[axis]) {
          uint32_t remaining=ENCODER_CLEAR_VERIFY_PULSES-encoder->recovery_pulses;
          encoder->recovery_pulses += delta_count<remaining?delta_count:remaining;
        }
        encoder->last_edge_tick_ms = now_ms;
        encoder->pulse_seen = 1U;
        /* Fault remains latched even if one noise pulse appears. */
        if (motor_command[axis] > 0)
        {
          encoder->assumed_direction = 1;
        }
        else if (motor_command[axis] < 0)
        {
          encoder->assumed_direction = -1;
        }
      }
      encoder->signed_rpm_estimate =
          (float)encoder->assumed_direction * encoder->rpm;
    }
    else
    {
      encoder->rejected_samples++;
      encoder->speed_valid=0U;
      encoder->recovery_pulses=0U;
      encoder->fault_flags |= 2U; /* implausible pulse rate; requires explicit clear */
    }

    encoder->previous_count = current_count;
    encoder->previous_tick_ms = now_ms;

    /* Operator-selected policy: zero/missing pulses never latch a fault or
       inhibit PWM. Keep RPM/count diagnostics; invalid-rate and overspeed
       checks are separate and unchanged. */
  }
}

static uint8_t WheelEncoder_CanClear(MotorAxis_t axis)
{
  const WheelEncoder_t *encoder=&wheel_encoder[axis];
  if(pid_enabled[axis] || motor_command[axis])return 0U;
  /* Each faulted wheel must produce one plausible revolution while disarmed.
     Keep that evidence until explicit clear, allowing sequential hand checks
     instead of requiring all three wheels to pulse within the same1second. */
  return !encoder->fault_flags ||
      (encoder->speed_valid && encoder->recovery_pulses>=ENCODER_CLEAR_VERIFY_PULSES);
}

static void WheelEncoders_Update(uint32_t now_ms)
{
  WheelEncoder_UpdateOne(&wheel_encoder[MOTOR_AXIS_ROLL], TIM1, now_ms);
  WheelEncoder_UpdateOne(&wheel_encoder[MOTOR_AXIS_PITCH], TIM8, now_ms);
  WheelEncoder_UpdateOne(&wheel_encoder[MOTOR_AXIS_YAW], TIM2, now_ms);
}

static void I2C1_BusRecover(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Release SCL and SDA, then clock out any unfinished slave transfer. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  App_Wait(2);

  for (uint8_t i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    App_Wait(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    App_Wait(1);
  }

  /* Generate a STOP condition. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  App_Wait(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  App_Wait(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  App_Wait(2);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  __HAL_RCC_I2C1_FORCE_RESET();
  App_Wait(1);
  __HAL_RCC_I2C1_RELEASE_RESET();

  MX_I2C1_Init();
  App_Wait(10);
}

static HAL_StatusTypeDef MPU6050_Write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(
      &hi2c1,
      mpu6050_address,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      &value,
      1,
      100);
}

static HAL_StatusTypeDef MPU6050_Read(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
  return HAL_I2C_Mem_Read(
      &hi2c1,
      mpu6050_address,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      data,
      length,
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_Init(uint8_t *who_am_i)
{
  const uint16_t addresses[] = {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH};
  HAL_StatusTypeDef status = HAL_ERROR;

  *who_am_i = 0;

  for (uint32_t i = 0; i < (sizeof(addresses) / sizeof(addresses[0])); i++)
  {
    mpu6050_address = addresses[i];
    status = MPU6050_Read(MPU6050_WHO_AM_I, who_am_i, 1);
    /* Genuine MPU6050 returns 0x68. Some pin/register-compatible modules
       sold as MPU6050 return 0x70, so accept both for basic accel/gyro use. */
    if ((status == HAL_OK) && ((*who_am_i == 0x68) || (*who_am_i == 0x70)))
    {
      break;
    }
    *who_am_i = 0;
  }

  if ((status != HAL_OK) || ((*who_am_i != 0x68) && (*who_am_i != 0x70)))
  {
    return HAL_ERROR;
  }

  /* Reset MPU6050 */
  if (MPU6050_Write(MPU6050_PWR_MGMT_1, 0x80) != HAL_OK)
  {
    return HAL_ERROR;
  }

  App_Wait(100);

  /* Đánh thức chip, dùng PLL gyro X làm clock */
  if (MPU6050_Write(MPU6050_PWR_MGMT_1, 0x01) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* DLPF bật: gyro sample base 1 kHz; divider 4 => 200 Hz. */
  if (MPU6050_Write(MPU6050_SMPLRT_DIV, 0x04) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* ~92/98Hz gyro LPF; lower delay than the former41/42Hz setting. */
  if (MPU6050_Write(MPU6050_CONFIG, 0x02) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* +/-1000dps,32.8LSB/dps. Saved gyro bias is already rad/s, not raw counts. */
  if (MPU6050_Write(MPU6050_GYRO_CONFIG, 0x10) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Accelerometer ±2g */
  if (MPU6050_Write(MPU6050_ACCEL_CONFIG, 0x00) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* WHO0x70 uses the separate accelerometer LPF register. Leaving it at
     reset default gave a much wider accel bandwidth than the gyro. */
  if(*who_am_i==0x70 && MPU6050_Write(MPU6500_ACCEL_CONFIG2,0x02)!=HAL_OK)return HAL_ERROR;
  uint8_t check=0;
  if(MPU6050_Read(MPU6050_GYRO_CONFIG,&check,1)!=HAL_OK || (check&0x1BU)!=0x10U)return HAL_ERROR;
  if(MPU6050_Read(MPU6050_CONFIG,&check,1)!=HAL_OK || (check&7U)!=2U)return HAL_ERROR;
  if(*who_am_i==0x70 && (MPU6050_Read(MPU6500_ACCEL_CONFIG2,&check,1)!=HAL_OK || (check&0x0FU)!=2U))return HAL_ERROR;

  App_Wait(50);

  return HAL_OK;
}

static HAL_StatusTypeDef MPU6050_ReadRaw(
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *temperature,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz)
{
  uint8_t data[14];

  if (MPU6050_Read(
          MPU6050_ACCEL_XOUT_H,
          data,
          sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *ax = (int16_t)((data[0] << 8) | data[1]);
  *ay = (int16_t)((data[2] << 8) | data[3]);
  *az = (int16_t)((data[4] << 8) | data[5]);

  *temperature = (int16_t)((data[6] << 8) | data[7]);

  *gx = (int16_t)((data[8] << 8) | data[9]);
  *gy = (int16_t)((data[10] << 8) | data[11]);
  *gz = (int16_t)((data[12] << 8) | data[13]);

  return HAL_OK;
}

static uint16_t ReadLeU16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static HAL_StatusTypeDef BMP280_ReadBytes(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
  return HAL_I2C_Mem_Read(
      &hi2c1,
      bmp280.address,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      data,
      length,
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BMP280_WriteByte(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(
      &hi2c1,
      bmp280.address,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      &value,
      1U,
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef BMP280_Init(void)
{
  const uint16_t addresses[] = {BMP280_ADDR_LOW, BMP280_ADDR_HIGH};
  uint8_t calibration[24];
  uint8_t id = 0U;

  memset(&bmp280, 0, sizeof(bmp280));

  for (uint32_t i = 0U; i < 2U; i++)
  {
    if (HAL_I2C_IsDeviceReady(
            &hi2c1,
            addresses[i],
            2U,
            OPTIONAL_I2C_TIMEOUT_MS) != HAL_OK)
    {
      continue;
    }

    bmp280.address = addresses[i];
    if ((BMP280_ReadBytes(BMP280_CHIP_ID_REG, &id, 1U) == HAL_OK)
        && (id == BMP280_CHIP_ID_VALUE))
    {
      break;
    }
    bmp280.address = 0U;
  }

  if (bmp280.address == 0U)
  {
    return HAL_ERROR;
  }

  bmp280.chip_id = id;
  if (BMP280_ReadBytes(
          BMP280_CALIB_REG,
          calibration,
          sizeof(calibration)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  bmp280.dig_t1 = ReadLeU16(&calibration[0]);
  bmp280.dig_t2 = (int16_t)ReadLeU16(&calibration[2]);
  bmp280.dig_t3 = (int16_t)ReadLeU16(&calibration[4]);
  bmp280.dig_p1 = ReadLeU16(&calibration[6]);
  bmp280.dig_p2 = (int16_t)ReadLeU16(&calibration[8]);
  bmp280.dig_p3 = (int16_t)ReadLeU16(&calibration[10]);
  bmp280.dig_p4 = (int16_t)ReadLeU16(&calibration[12]);
  bmp280.dig_p5 = (int16_t)ReadLeU16(&calibration[14]);
  bmp280.dig_p6 = (int16_t)ReadLeU16(&calibration[16]);
  bmp280.dig_p7 = (int16_t)ReadLeU16(&calibration[18]);
  bmp280.dig_p8 = (int16_t)ReadLeU16(&calibration[20]);
  bmp280.dig_p9 = (int16_t)ReadLeU16(&calibration[22]);

  if (bmp280.dig_p1 == 0U)
  {
    return HAL_ERROR;
  }

  /* 125-ms standby, IIR filter x4. */
  if (BMP280_WriteByte(BMP280_CONFIG_REG, 0x48U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* Temperature x2, pressure x16, normal mode. */
  if (BMP280_WriteByte(BMP280_CTRL_MEAS_REG, 0x57U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef BMP280_ReadSample(void)
{
  uint8_t data[6];
  int32_t adc_temperature;
  int32_t adc_pressure;
  int32_t var1_t;
  int32_t var2_t;
  int32_t delta_t;
  int64_t var1_p;
  int64_t var2_p;
  int64_t pressure_q24_8;

  if (BMP280_ReadBytes(BMP280_PRESS_MSB_REG, data, sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  adc_pressure = ((int32_t)data[0] << 12)
               | ((int32_t)data[1] << 4)
               | ((int32_t)data[2] >> 4);
  adc_temperature = ((int32_t)data[3] << 12)
                  | ((int32_t)data[4] << 4)
                  | ((int32_t)data[5] >> 4);

  if ((adc_pressure == 0x80000L) || (adc_temperature == 0x80000L))
  {
    return HAL_BUSY;
  }

  var1_t = ((((adc_temperature >> 3)
             - ((int32_t)bmp280.dig_t1 * 2))
            * (int32_t)bmp280.dig_t2) >> 11);
  delta_t = (adc_temperature >> 4) - (int32_t)bmp280.dig_t1;
  var2_t = (((delta_t * delta_t) >> 12) * (int32_t)bmp280.dig_t3) >> 14;
  bmp280.t_fine = var1_t + var2_t;
  bmp280.temperature_centi_c = (bmp280.t_fine * 5 + 128) >> 8;

  var1_p = (int64_t)bmp280.t_fine - 128000LL;
  var2_p = var1_p * var1_p * (int64_t)bmp280.dig_p6;
  var2_p += var1_p * (int64_t)bmp280.dig_p5 * (1LL << 17);
  var2_p += (int64_t)bmp280.dig_p4 * (1LL << 35);
  var1_p = ((var1_p * var1_p * (int64_t)bmp280.dig_p3) >> 8)
         + ((var1_p * (int64_t)bmp280.dig_p2) * (1LL << 12));
  var1_p = ((((1LL << 47) + var1_p) * (int64_t)bmp280.dig_p1) >> 33);

  if (var1_p == 0LL)
  {
    return HAL_ERROR;
  }

  pressure_q24_8 = 1048576LL - (int64_t)adc_pressure;
  pressure_q24_8 = (((pressure_q24_8 << 31) - var2_p) * 3125LL) / var1_p;
  var1_p = ((int64_t)bmp280.dig_p9
            * (pressure_q24_8 >> 13)
            * (pressure_q24_8 >> 13)) >> 25;
  var2_p = ((int64_t)bmp280.dig_p8 * pressure_q24_8) >> 19;
  pressure_q24_8 = ((pressure_q24_8 + var1_p + var2_p) >> 8)
                 + ((int64_t)bmp280.dig_p7 * 16LL);

  bmp280.pressure_pa = (uint32_t)(pressure_q24_8 >> 8);
  bmp280.valid = 1U;
  last_env_sample_ms[0]=HAL_GetTick();
  bmp280.failures = 0U;
  return HAL_OK;
}

static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value)
{
  uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)value};
  return HAL_I2C_Mem_Write(
      &hi2c1,
      INA219_ADDR,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      data,
      sizeof(data),
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value)
{
  uint8_t data[2];
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      &hi2c1,
      INA219_ADDR,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      data,
      sizeof(data),
      OPTIONAL_I2C_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    *value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  }
  return status;
}

static HAL_StatusTypeDef INA219_Init(void)
{
  memset(&ina219, 0, sizeof(ina219));
  if (HAL_I2C_IsDeviceReady(
          &hi2c1,
          INA219_ADDR,
          2U,
          OPTIONAL_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 32-V bus range, 320-mV shunt range, 12-bit continuous shunt+bus. */
  return INA219_WriteRegister(INA219_CONFIG_REG, 0x399FU);
}

static HAL_StatusTypeDef INA219_ReadSample(void)
{
  uint16_t shunt_raw_u16;
  uint16_t bus_raw;
  int16_t shunt_raw;

  if (INA219_ReadRegister(INA219_SHUNT_REG, &shunt_raw_u16) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (INA219_ReadRegister(INA219_BUS_REG, &bus_raw) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((bus_raw & 0x0001U) != 0U)
  {
    return HAL_ERROR;
  }

  shunt_raw = (int16_t)shunt_raw_u16;
  ina219.shunt_uv = (int32_t)shunt_raw * 10L;
  ina219.bus_mv = (int32_t)((bus_raw >> 3) * 4U);
  ina219.current_ma_x10 =
      ((int32_t)shunt_raw * 100L) / INA219_SHUNT_MILLIOHMS;
  ina219.power_mw =
      (ina219.bus_mv * ina219.current_ma_x10) / 10000L;
  ina219.valid = 1U;
  last_env_sample_ms[1]=HAL_GetTick();
  ina219.failures = 0U;
  return HAL_OK;
}

static uint8_t AHT30_Crc8(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0xFFU;

  for (uint32_t byte = 0U; byte < length; byte++)
  {
    crc ^= data[byte];
    for (uint32_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 0x80U)
          ? (uint8_t)((crc << 1) ^ 0x31U)
          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static HAL_StatusTypeDef AHT30_Init(void)
{
  memset(&aht30, 0, sizeof(aht30));
  return HAL_I2C_IsDeviceReady(
      &hi2c1,
      AHT30_ADDR,
      2U,
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AHT30_StartMeasurement(uint32_t now)
{
  uint8_t command[3] = {0xACU, 0x33U, 0x00U};
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      &hi2c1,
      AHT30_ADDR,
      command,
      sizeof(command),
      OPTIONAL_I2C_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    aht30.trigger_tick = now;
    aht30.measurement_pending = 1U;
  }
  return status;
}

static HAL_StatusTypeDef AHT30_ReadMeasurement(void)
{
  uint8_t data[7];
  uint32_t raw_humidity;
  uint32_t raw_temperature;

  if (HAL_I2C_Master_Receive(
          &hi2c1,
          AHT30_ADDR,
          data,
          sizeof(data),
          OPTIONAL_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((data[0] & 0x80U) != 0U)
  {
    return HAL_BUSY;
  }
  if (AHT30_Crc8(data, 6U) != data[6])
  {
    return HAL_ERROR;
  }

  raw_humidity = ((uint32_t)data[1] << 12)
               | ((uint32_t)data[2] << 4)
               | ((uint32_t)data[3] >> 4);
  raw_temperature = (((uint32_t)data[3] & 0x0FU) << 16)
                  | ((uint32_t)data[4] << 8)
                  | (uint32_t)data[5];

  aht30.humidity_milli_percent =
      (uint32_t)(((uint64_t)raw_humidity * 100000ULL) / 1048576ULL);
  aht30.temperature_milli_c =
      (int32_t)(((uint64_t)raw_temperature * 200000ULL) / 1048576ULL)
      - 50000L;
  aht30.valid = 1U;
  last_env_sample_ms[2]=HAL_GetTick();
  aht30.failures = 0U;
  return HAL_OK;
}

/* Perform at most one slow-sensor service operation per pass. These short,
   bounded I2C accesses run after the 200-Hz attitude/PID control work. */
static void SlowSensors_Service(uint32_t now)
{
  HAL_StatusTypeDef status;

  if ((aht30_status == HAL_OK) && (aht30.measurement_pending != 0U)
      && ((now - aht30.trigger_tick) >= AHT30_MEASUREMENT_TIME_MS))
  {
    status = AHT30_ReadMeasurement();
    if (status == HAL_BUSY && now-aht30.trigger_tick<200U)
    {
      return;
    }
    aht30.measurement_pending = 0U;
    last_aht30_tick = now;
    if (status != HAL_OK)
    {
      aht30.failures++;
      if (aht30.failures >= 5U)
      {
        aht30_status = HAL_ERROR;
      }
    }
    return;
  }

  if ((aht30_status == HAL_OK) && (aht30.measurement_pending == 0U)
      && ((now - last_aht30_tick) >= AHT30_UPDATE_PERIOD_MS))
  {
    if (AHT30_StartMeasurement(now) != HAL_OK)
    {
      aht30.failures++;
      last_aht30_tick = now;
      if (aht30.failures >= 5U)
      {
        aht30_status = HAL_ERROR;
      }
    }
    return;
  }

  if ((ina219_status == HAL_OK)
      && ((now - last_ina219_tick) >= INA219_UPDATE_PERIOD_MS))
  {
    last_ina219_tick = now;
    if (INA219_ReadSample() != HAL_OK)
    {
      ina219.failures++;
      if (ina219.failures >= 5U)
      {
        ina219_status = HAL_ERROR;
      }
    }
    return;
  }

  if ((bmp280_status == HAL_OK)
      && ((now - last_bmp280_tick) >= BMP280_UPDATE_PERIOD_MS))
  {
    last_bmp280_tick = now;
    status = BMP280_ReadSample();
    if ((status != HAL_OK) && (status != HAL_BUSY))
    {
      bmp280.failures++;
      if (bmp280.failures >= 5U)
      {
        bmp280_status = HAL_ERROR;
      }
    }
  }
}

static HAL_StatusTypeDef GY511_MagWrite(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(
      &hi2c1,
      gy511_mag_address,
      reg,
      I2C_MEMADD_SIZE_8BIT,
      &value,
      1,
      OPTIONAL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef GY511_MagInit(void)
{
  const uint16_t candidates[] = {LSM303D_ADDR_LOW, LSM303D_ADDR_HIGH};
  gy511_mag_type = GY511_MAG_NONE;
  gy511_mag_address = 0U;
  gy511_mag_who_am_i = 0U;

  for (uint32_t i = 0; i < 2U; i++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, candidates[i], 3, 100) != HAL_OK)
    {
      continue;
    }

    uint8_t who = 0U;
    HAL_StatusTypeDef who_status = HAL_I2C_Mem_Read(
        &hi2c1,
        candidates[i],
        LSM303D_WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who,
        1,
        100);

    if ((who_status == HAL_OK) && (who == LSM303D_WHO_AM_I_VALUE))
    {
      gy511_mag_type = GY511_MAG_LSM303D;
      gy511_mag_address = candidates[i];
      gy511_mag_who_am_i = who;
      break;
    }

    /* A discrete LSM303DLHC magnetometer has no 0x49 WHO_AM_I and uses 0x1E. */
    if (candidates[i] == LSM303DLHC_MAG_ADDR)
    {
      gy511_mag_type = GY511_MAG_LSM303DLHC;
      gy511_mag_address = candidates[i];
      gy511_mag_who_am_i = who;
    }
  }

  if (gy511_mag_type == GY511_MAG_NONE)
  {
    return HAL_ERROR;
  }

  if (gy511_mag_type == GY511_MAG_LSM303D)
  {
    /* High-resolution magnetometer, 50 Hz, +/-4 gauss, continuous mode. */
    if (GY511_MagWrite(LSM303D_CTRL5, 0x70) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (GY511_MagWrite(LSM303D_CTRL6, 0x20) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (GY511_MagWrite(LSM303D_CTRL7, 0x00) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  else
  {
    /* LSM303DLHC: 75 Hz, +/-1.3 gauss, continuous mode. */
    if (GY511_MagWrite(LSM303DLHC_CRA_REG, 0x18) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (GY511_MagWrite(LSM303DLHC_CRB_REG, 0x20) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if (GY511_MagWrite(LSM303DLHC_MR_REG, 0x00) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  App_Wait(20);
  return HAL_OK;
}

static HAL_StatusTypeDef GY511_MagReadRaw(
    int16_t *mx,
    int16_t *my,
    int16_t *mz)
{
  uint8_t status = 0;
  uint8_t data[6];

  if ((gy511_mag_type == GY511_MAG_NONE) || (gy511_mag_address == 0U))
  {
    return HAL_ERROR;
  }

  uint8_t status_register =
      (gy511_mag_type == GY511_MAG_LSM303D)
      ? LSM303D_STATUS_M_REG
      : LSM303DLHC_STATUS_REG;
  uint8_t ready_mask =
      (gy511_mag_type == GY511_MAG_LSM303D) ? 0x08U : 0x01U;

  if (HAL_I2C_Mem_Read(
          &hi2c1,
          gy511_mag_address,
          status_register,
          I2C_MEMADD_SIZE_8BIT,
          &status,
          1,
          OPTIONAL_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if ((status & ready_mask) == 0U)
  {
    return HAL_BUSY;
  }

  uint8_t first_data_register =
      (gy511_mag_type == GY511_MAG_LSM303D)
      ? (LSM303D_OUT_X_L_M | 0x80U)
      : LSM303DLHC_OUT_X_H;
  if (HAL_I2C_Mem_Read(
          &hi2c1,
          gy511_mag_address,
          first_data_register,
          I2C_MEMADD_SIZE_8BIT,
          data,
          sizeof(data),
          OPTIONAL_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (gy511_mag_type == GY511_MAG_LSM303D)
  {
    /* LSM303D order is little-endian X, Y, Z. */
    *mx = (int16_t)((data[1] << 8) | data[0]);
    *my = (int16_t)((data[3] << 8) | data[2]);
    *mz = (int16_t)((data[5] << 8) | data[4]);
  }
  else
  {
    /* LSM303DLHC/HMC5883 order is big-endian X, Z, Y. */
    *mx = (int16_t)((data[0] << 8) | data[1]);
    *mz = (int16_t)((data[2] << 8) | data[3]);
    *my = (int16_t)((data[4] << 8) | data[5]);
  }

  /* -4096 is a DLHC/HMC overflow code, not an LSM303D sentinel. */
  if (gy511_mag_type==GY511_MAG_LSM303DLHC &&
      ((*mx == -4096) || (*my == -4096) || (*mz == -4096)))
  {
    mag_reject_reason=5U;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/* Recover only the optional magnetometer, one short I2C transaction per pass.
   Never recover/recalibrate the IMU or reset attitude/target for a mag fault. */
static uint8_t Magnetometer_RecoveryService(uint32_t now)
{
  static uint32_t attempt_tick;
  static uint8_t step;
  if(!step) {
    if(mag_status==HAL_OK && now-last_mag_sample_ms<=1000U)return 0U;
    if(now-attempt_tick<1000U)return 0U;
    attempt_tick=now;step=1U;
  }
  HAL_StatusTypeDef status;uint8_t who=0;
  if(step<=2U) {
    uint16_t address=step==1U?LSM303D_ADDR_LOW:LSM303D_ADDR_HIGH;
    status=HAL_I2C_Mem_Read(&hi2c1,address,LSM303D_WHO_AM_I_REG,I2C_MEMADD_SIZE_8BIT,&who,1,OPTIONAL_I2C_TIMEOUT_MS);
    if(status==HAL_OK && (who==LSM303D_WHO_AM_I_VALUE || step==2U)) {
      gy511_mag_type=who==LSM303D_WHO_AM_I_VALUE?GY511_MAG_LSM303D:GY511_MAG_LSM303DLHC;
      gy511_mag_address=address;gy511_mag_who_am_i=who;step=3U;
    } else if(step==1U)step=2U;
    else step=0U;
    return 1U;
  }
  const uint8_t regs_d[3]={LSM303D_CTRL5,LSM303D_CTRL6,LSM303D_CTRL7};
  const uint8_t values_d[3]={0x70,0x20,0x00};
  const uint8_t regs_h[3]={LSM303DLHC_CRA_REG,LSM303DLHC_CRB_REG,LSM303DLHC_MR_REG};
  const uint8_t values_h[3]={0x18,0x20,0x00};
  unsigned index=step-3U;
  status=GY511_MagWrite(gy511_mag_type==GY511_MAG_LSM303D?regs_d[index]:regs_h[index],
      gy511_mag_type==GY511_MAG_LSM303D?values_d[index]:values_h[index]);
  if(status!=HAL_OK){step=0U;mag_status=HAL_ERROR;mag_io_errors++;}
  else if(++step>=6U){step=0U;mag_status=HAL_OK;}
  return 1U;
}

static void GY511_MagToBody(
    int16_t mx,
    int16_t my,
    int16_t mz,
    float body_mag[3])
{
  float calibrated_sensor[3] =
  {
    ((float)mx - magnetometer.offset[0]) * magnetometer.scale[0],
    ((float)my - magnetometer.offset[1]) * magnetometer.scale[1],
    ((float)mz - magnetometer.offset[2]) * magnetometer.scale[2]
  };

  body_mag[0] = MAG_BODY_X_SIGN
              * calibrated_sensor[MAG_BODY_X_SENSOR_AXIS];
  body_mag[1] = MAG_BODY_Y_SIGN
              * calibrated_sensor[MAG_BODY_Y_SENSOR_AXIS];
  body_mag[2] = MAG_BODY_Z_SIGN
              * calibrated_sensor[MAG_BODY_Z_SENSOR_AXIS];
  ADCS_LegacyToBody(body_mag,body_mag);
}

static HAL_StatusTypeDef GY511_MagCalibrate(void)
{
  int16_t minimum[3] = {32767, 32767, 32767};
  int16_t maximum[3] = {-32768, -32768, -32768};
  uint32_t valid_samples = 0;
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_report_tick = start_tick;
  char message[192];

  Motors_StopAll();
  {
    static uint8_t start_message[] =
        "MAG CAL: ROTATE SYSTEM SLOWLY THROUGH ALL DIRECTIONS FOR 20 SECONDS\r\n";
    TelemetryProtocol_Event(start_message, sizeof(start_message) - 1U);
  }

  while ((HAL_GetTick() - start_tick) < MAG_CALIBRATION_TIME_MS)
  {
    if(emergency_stop)return HAL_ERROR;
    int16_t raw[3];
    HAL_StatusTypeDef status = GY511_MagReadRaw(&raw[0], &raw[1], &raw[2]);
    if (status == HAL_OK)
    {
      for (uint32_t axis = 0; axis < 3U; axis++)
      {
        if (raw[axis] < minimum[axis])
        {
          minimum[axis] = raw[axis];
        }
        if (raw[axis] > maximum[axis])
        {
          maximum[axis] = raw[axis];
        }
      }
      valid_samples++;
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_report_tick) >= 5000U)
    {
      last_report_tick = now;
      int length = snprintf(
          message,
          sizeof(message),
          "MAG CAL PROGRESS %lu/20 s MIN=%d,%d,%d MAX=%d,%d,%d\r\n",
          (unsigned long)((now - start_tick) / 1000U),
          minimum[0], minimum[1], minimum[2],
          maximum[0], maximum[1], maximum[2]);
      TelemetryProtocol_Event((uint8_t *)message, length);
    }
    App_Wait(14);
  }

  float half_range[3],candidate_offset[3];
  for (uint32_t axis = 0; axis < 3U; axis++)
  {
    candidate_offset[axis] =
        0.5f * ((float)maximum[axis] + (float)minimum[axis]);
    half_range[axis] =
        0.5f * ((float)maximum[axis] - (float)minimum[axis]);
    if (half_range[axis] < 50.0f)
    {
      return HAL_ERROR;
    }
  }

  if (valid_samples < 500U)
  {
    return HAL_ERROR;
  }

  float average_half_range =
      (half_range[0] + half_range[1] + half_range[2]) / 3.0f;
  for (uint32_t axis = 0; axis < 3U; axis++)
  {
    /* Diagonal soft-iron correction. Full cross-axis correction would require
       an ellipsoid fit off-board; this is robust enough for initial use. */
    magnetometer.scale[axis] = average_half_range / half_range[axis];
    magnetometer.offset[axis] = candidate_offset[axis];
  }
  magnetometer.calibrated = 1U;
  magnetometer.reference_valid = 0U;
  magnetometer.rejected_samples = 0U;
  Calibration_Save(CALIBRATION_MAG);

  int length = snprintf(
      message,
      sizeof(message),
      "MAG CAL DONE OFF_x10=%ld,%ld,%ld SCALE_u6=%ld,%ld,%ld\r\n",
      (long)(magnetometer.offset[0] * 10.0f),
      (long)(magnetometer.offset[1] * 10.0f),
      (long)(magnetometer.offset[2] * 10.0f),
      (long)(magnetometer.scale[0] * 1000000.0f),
      (long)(magnetometer.scale[1] * 1000000.0f),
      (long)(magnetometer.scale[2] * 1000000.0f));
  TelemetryProtocol_Event((uint8_t *)message, length);
  return HAL_OK;
}

static void SensorToBodyAccel(
    float sensor_x_raw,
    float sensor_y_raw,
    float sensor_z_raw,
    float body_accel_raw[3])
{
  float calibrated_sensor_x =
      (sensor_x_raw - accel_offset_raw[0]) * accel_scale[0];
  float calibrated_sensor_y =
      (sensor_y_raw - accel_offset_raw[1]) * accel_scale[1];
  float calibrated_sensor_z =
      (sensor_z_raw - accel_offset_raw[2]) * accel_scale[2];

  /* Historical sensor mounting first; apply measured body-basis rotation below. */
  body_accel_raw[0] = -calibrated_sensor_x;
  body_accel_raw[1] =  calibrated_sensor_y;
  body_accel_raw[2] = -calibrated_sensor_z;
  ADCS_LegacyToBody(body_accel_raw,body_accel_raw);
}

static void SensorToBodyGyroRad(
    float sensor_x_raw,
    float sensor_y_raw,
    float sensor_z_raw,
    float body_gyro_rad_s[3])
{
  body_gyro_rad_s[0] = -(sensor_x_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  body_gyro_rad_s[1] =  (sensor_y_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  body_gyro_rad_s[2] = -(sensor_z_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
  ADCS_LegacyToBody(body_gyro_rad_s,body_gyro_rad_s);
}

static HAL_StatusTypeDef Sensor_Calibrate(
    float gyro_bias_rad_s[3],
    float accel_average_raw[3])
{
  int64_t gyro_sum[3] = {0, 0, 0};
  int64_t accel_sum[3] = {0, 0, 0};
  int16_t gyro_min[3] = {32767, 32767, 32767};
  int16_t gyro_max[3] = {-32768, -32768, -32768};
  int16_t accel_min[3] = {32767, 32767, 32767};
  int16_t accel_max[3] = {-32768, -32768, -32768};
  uint32_t valid_samples = 0;
  int16_t ax, ay, az, temperature, gx, gy, gz;
  char message[160];

  Motors_StopAll();

  {
    static uint8_t start_message[] =
        "CAL STATUS TYPE=GYRO PHASE=SAMPLING KEEP_SYSTEM_STILL\r\n";
    TelemetryProtocol_Event(start_message, sizeof(start_message) - 1U);
  }

  for (uint32_t i = 0; i < 800U; i++)
  {
    if(emergency_stop)return HAL_ERROR;
    if (MPU6050_ReadRaw(&ax, &ay, &az, &temperature, &gx, &gy, &gz) == HAL_OK)
    {
      int16_t accel_sample[3] = {ax, ay, az};
      int16_t gyro_sample[3] = {gx, gy, gz};
      accel_sum[0] += ax;
      accel_sum[1] += ay;
      accel_sum[2] += az;
      gyro_sum[0] += gx;
      gyro_sum[1] += gy;
      gyro_sum[2] += gz;
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        if (accel_sample[axis] < accel_min[axis]) accel_min[axis] = accel_sample[axis];
        if (accel_sample[axis] > accel_max[axis]) accel_max[axis] = accel_sample[axis];
        if (gyro_sample[axis] < gyro_min[axis]) gyro_min[axis] = gyro_sample[axis];
        if (gyro_sample[axis] > gyro_max[axis]) gyro_max[axis] = gyro_sample[axis];
      }
      valid_samples++;
    }
    App_Wait(5);
  }

  if (valid_samples < 700U)
  {
    static uint8_t sample_error[] =
        "CAL ERROR TYPE=GYRO REASON=I2C_OR_TOO_FEW_SAMPLES\r\n";
    TelemetryProtocol_Event(sample_error, sizeof(sample_error) - 1U);
    return HAL_ERROR;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    if (((int32_t)gyro_max[axis] - (int32_t)gyro_min[axis]) > GYRO_CAL_MAX_RANGE_RAW
        || ((int32_t)accel_max[axis] - (int32_t)accel_min[axis]) > 1800)
    {
      int length = snprintf(
          message,
          sizeof(message),
          "CAL ERROR TYPE=GYRO REASON=MOTION AXIS=%lu GRANGE=%ld ARANGE=%ld\r\n",
          (unsigned long)axis,
          (long)((int32_t)gyro_max[axis] - (int32_t)gyro_min[axis]),
          (long)((int32_t)accel_max[axis] - (int32_t)accel_min[axis]));
      TelemetryProtocol_Event((uint8_t *)message, length);
      return HAL_ERROR;
    }
  }

  SensorToBodyAccel(
      (float)accel_sum[0] / (float)valid_samples,
      (float)accel_sum[1] / (float)valid_samples,
      (float)accel_sum[2] / (float)valid_samples,
      accel_average_raw);
  SensorToBodyGyroRad(
      (float)gyro_sum[0] / (float)valid_samples,
      (float)gyro_sum[1] / (float)valid_samples,
      (float)gyro_sum[2] / (float)valid_samples,
      gyro_bias_rad_s);
  memcpy(calibration_gyro_bias,gyro_bias_rad_s,sizeof(calibration_gyro_bias));
  calibration_gyro_valid=1U;
  Calibration_Save(CALIBRATION_GYRO);

  {
    int length = snprintf(
        message,
        sizeof(message),
        "CAL DONE TYPE=GYRO BIAS_URAD=%ld,%ld,%ld\r\n",
        (long)(gyro_bias_rad_s[0] * 1000000.0f),
        (long)(gyro_bias_rad_s[1] * 1000000.0f),
        (long)(gyro_bias_rad_s[2] * 1000000.0f));
    TelemetryProtocol_Event((uint8_t *)message, length);
  }

  return HAL_OK;
}

static void AccelSixPoint_Start(void)
{
  /* Keep the operator PID request across calibration, faults and recovery. */
  memset(&accel_calibration, 0, sizeof(accel_calibration));
  accel_calibration.active = 1U;
  Motors_StopAll();
  {
    static uint8_t message[] =
        "CAL STATUS TYPE=ACCEL PHASE=READY CAPTURED=0/6 "
        "INSTRUCTION=PLACE_ONE_FACE_UP_KEEP_STILL_THEN_CAPTURE\r\n";
    TelemetryProtocol_Event(message, sizeof(message) - 1U);
  }
}

/* Returns 1 when all six faces are complete, 0 after one valid face,
   and -1 when the capture must be repeated. */
static int32_t AccelSixPoint_Capture(void)
{
  if (accel_calibration.active == 0U)
  {
    static uint8_t message[] =
        "CAL ERROR TYPE=ACCEL REASON=NOT_STARTED\r\n";
    TelemetryProtocol_Event(message, sizeof(message) - 1U);
    return -1;
  }

  int64_t sum[3] = {0, 0, 0};
  int16_t minimum[3] = {32767, 32767, 32767};
  int16_t maximum[3] = {-32768, -32768, -32768};
  uint32_t valid_samples = 0U;
  int16_t ax, ay, az, temperature, gx, gy, gz;

  Motors_StopAll();
  {
    static uint8_t message[] =
        "CAL STATUS TYPE=ACCEL PHASE=SAMPLING KEEP_SYSTEM_STILL\r\n";
    TelemetryProtocol_Event(message, sizeof(message) - 1U);
  }

  for (uint32_t sample = 0U; sample < 500U; sample++)
  {
    if(emergency_stop || !accel_calibration.active)return -1;
    if (MPU6050_ReadRaw(&ax, &ay, &az, &temperature, &gx, &gy, &gz) == HAL_OK)
    {
      int16_t value[3] = {ax, ay, az};
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        sum[axis] += value[axis];
        if (value[axis] < minimum[axis]) minimum[axis] = value[axis];
        if (value[axis] > maximum[axis]) maximum[axis] = value[axis];
      }
      valid_samples++;
    }
    App_Wait(4);
  }

  if (valid_samples < 450U)
  {
    static uint8_t message[] =
        "CAL ERROR TYPE=ACCEL REASON=I2C_OR_TOO_FEW_SAMPLES\r\n";
    TelemetryProtocol_Event(message, sizeof(message) - 1U);
    return -1;
  }

  float average[3];
  uint32_t dominant_axis = 0U;
  float dominant_value = 0.0f;
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    int32_t range = (int32_t)maximum[axis] - (int32_t)minimum[axis];
    if (range > 1800)
    {
      char message[112];
      int length = snprintf(
          message,
          sizeof(message),
          "CAL ERROR TYPE=ACCEL REASON=MOTION AXIS=%lu RANGE=%ld\r\n",
          (unsigned long)axis,
          (long)range);
      TelemetryProtocol_Event((uint8_t *)message, length);
      return -1;
    }
    average[axis] = (float)sum[axis] / (float)valid_samples;
    if (fabsf(average[axis]) > dominant_value)
    {
      dominant_value = fabsf(average[axis]);
      dominant_axis = axis;
    }
  }

  if (dominant_value < (0.72f * ACCEL_LSB_PER_G))
  {
    static uint8_t message[] =
        "CAL ERROR TYPE=ACCEL REASON=FACE_NOT_LEVEL\r\n";
    TelemetryProtocol_Event(message, sizeof(message) - 1U);
    return -1;
  }

  uint32_t sign_index = (average[dominant_axis] >= 0.0f) ? 1U : 0U;
  if (accel_calibration.captured[dominant_axis][sign_index] != 0U)
  {
    char message[112];
    int length = snprintf(
        message,
        sizeof(message),
        "CAL ERROR TYPE=ACCEL REASON=DUPLICATE_FACE FACE=%c%c\r\n",
        (sign_index != 0U) ? '+' : '-',
        (char)('X' + dominant_axis));
    TelemetryProtocol_Event((uint8_t *)message, length);
    return -1;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    accel_calibration.face_mean[dominant_axis][sign_index][axis] = average[axis];
  }
  accel_calibration.captured[dominant_axis][sign_index] = 1U;
  accel_calibration.count++;

  {
    char message[160];
    int length = snprintf(
        message,
        sizeof(message),
        "CAL STATUS TYPE=ACCEL PHASE=FACE_SAVED FACE=%c%c CAPTURED=%u/6 "
        "INSTRUCTION=TURN_TO_A_NEW_FACE\r\n",
        (sign_index != 0U) ? '+' : '-',
        (char)('X' + dominant_axis),
        accel_calibration.count);
    TelemetryProtocol_Event((uint8_t *)message, length);
  }

  if (accel_calibration.count < 6U)
  {
    return 0;
  }

  float new_offset[3];
  float new_scale[3];
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    float negative = accel_calibration.face_mean[axis][0][axis];
    float positive = accel_calibration.face_mean[axis][1][axis];
    float span = positive - negative;
    if (span < (1.55f * ACCEL_LSB_PER_G))
    {
      accel_calibration.active = 0U;
      static uint8_t message[] =
          "CAL ERROR TYPE=ACCEL REASON=INVALID_SIX_FACE_SPAN\r\n";
      TelemetryProtocol_Event(message, sizeof(message) - 1U);
      return -1;
    }
    new_offset[axis] = 0.5f * (positive + negative);
    new_scale[axis] = (2.0f * ACCEL_LSB_PER_G) / span;
    if ((new_scale[axis] < 0.75f) || (new_scale[axis] > 1.25f))
    {
      accel_calibration.active = 0U;
      static uint8_t message[] =
          "CAL ERROR TYPE=ACCEL REASON=INVALID_SCALE\r\n";
      TelemetryProtocol_Event(message, sizeof(message) - 1U);
      return -1;
    }
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    accel_offset_raw[axis] = new_offset[axis];
    accel_scale[axis] = new_scale[axis];
  }
  accel_calibration.active = 0U;

  {
    char message[192];
    int length = snprintf(
        message,
        sizeof(message),
        "CAL DONE TYPE=ACCEL OFF_x10=%ld,%ld,%ld SCALE_u6=%ld,%ld,%ld\r\n",
        (long)(accel_offset_raw[0] * 10.0f),
        (long)(accel_offset_raw[1] * 10.0f),
        (long)(accel_offset_raw[2] * 10.0f),
        (long)(accel_scale[0] * 1000000.0f),
        (long)(accel_scale[1] * 1000000.0f),
        (long)(accel_scale[2] * 1000000.0f));
    TelemetryProtocol_Event((uint8_t *)message, length);
  }
  Calibration_Save(CALIBRATION_ACCEL);
  return 1;
}

/* newlib-nano does not enable scanf floating-point conversion unless the
   linker pulls in _scanf_float.  This small decimal parser keeps the command
   reliable without adding the large scanf/strtof floating-point library. */
static UsbCommand_t UsbCommand_Poll(void)
{
  ParsedCommand command;
  App_CommunicationService(0);
  if(!CommandParser_Pop(&command))return USB_COMMAND_NONE;
  snprintf(last_received_command,sizeof(last_received_command),"%s",command.text);
  switch(command.type) {
    case USB_COMMAND_SET_ATTITUDE:memcpy(pending_setpoint_deg,command.values,sizeof(command.values));break;
    case USB_COMMAND_SET_WHEEL_RPM:memcpy(pending_wheel_rpm,command.values,sizeof(command.values));break;
    case USB_COMMAND_SET_RPM_LIMIT:memcpy(pending_rpm_limit,command.values,sizeof(command.values));break;
    case USB_COMMAND_SET_WHEEL_PID:pending_wheel_axis=command.axis;memcpy(pending_wheel_pid_gain,command.values,sizeof(command.values));break;
    case USB_COMMAND_PID:pending_pid_axis=command.axis;pending_pid_enable=command.enabled;break;
    default:break;
  }
  return command.type;
}

static void QuaternionNormalize(float q[4])
{
  float norm = sqrtf(
      q[0] * q[0] + q[1] * q[1] +
      q[2] * q[2] + q[3] * q[3]);

  if (!isfinite(norm) || norm < 1.0e-9f)
  {
    fusion_status=HAL_ERROR;
    /* Keep the operator PID request across calibration, faults and recovery. */
    Motors_StopAll();
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }

  float inverse_norm = 1.0f / norm;
  for (uint32_t i = 0; i < 4U; i++)
  {
    q[i] *= inverse_norm;
  }
}

static void QuaternionToEuler(
    const float q[4],
    float *roll_deg,
    float *pitch_deg,
    float *yaw_deg)
{
  float sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
  sin_pitch = ClampFloat(sin_pitch, -1.0f, 1.0f);

  *roll_deg = atan2f(
      2.0f * (q[0] * q[1] + q[2] * q[3]),
      1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
  *pitch_deg = asinf(sin_pitch) * RAD_TO_DEG;
  *yaw_deg = atan2f(
      2.0f * (q[0] * q[3] + q[1] * q[2]),
      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
}

static void __attribute__((unused)) EulerToOutputQuaternion(
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    float output_q[4])
{
  float half_roll = 0.5f * roll_deg * DEG_TO_RAD;
  float half_pitch = 0.5f * pitch_deg * DEG_TO_RAD;
  float half_yaw = 0.5f * yaw_deg * DEG_TO_RAD;
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);
  float cy = cosf(half_yaw);
  float sy = sinf(half_yaw);

  /* ZYX aerospace convention: yaw, then pitch, then roll. */
  output_q[0] = cr * cp * cy + sr * sp * sy;
  output_q[1] = sr * cp * cy - cr * sp * sy;
  output_q[2] = cr * sp * cy + sr * cp * sy;
  output_q[3] = cr * cp * sy - sr * sp * cy;
  QuaternionNormalize(output_q);
}

static int Matrix3Inverse(const float matrix[3][3], float inverse[3][3])
{
  float determinant =
      matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

  if (fabsf(determinant) < 1.0e-9f)
  {
    return 0;
  }

  float inverse_determinant = 1.0f / determinant;
  inverse[0][0] =  (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inverse_determinant;
  inverse[0][1] =  (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inverse_determinant;
  inverse[0][2] =  (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inverse_determinant;
  inverse[1][0] =  (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inverse_determinant;
  inverse[1][1] =  (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inverse_determinant;
  inverse[1][2] =  (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inverse_determinant;
  inverse[2][0] =  (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inverse_determinant;
  inverse[2][1] =  (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inverse_determinant;
  inverse[2][2] =  (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inverse_determinant;
  return 1;
}

static void AttitudeEKF_Init(
    const float gyro_bias_rad_s[3],
    const float accel_average_raw[3])
{
  float retained_yaw=(control_targets_initialized && isfinite(attitude_ekf.yaw_deg))
      ? attitude_ekf.yaw_deg*DEG_TO_RAD : 0.0f;
  memset(&attitude_ekf, 0, sizeof(attitude_ekf));
  last_heading_update_ms=0U;
  clean_heading_samples=0U;
  magnetic_heading_quality=0U;
  mag_reference_epoch++;

  float ax = accel_average_raw[0];
  float ay = accel_average_raw[1];
  float az = accel_average_raw[2];
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float half_roll = 0.5f * roll;
  float half_pitch = 0.5f * pitch;
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);

  /* Initial yaw is defined as zero. Quaternion maps body to world. */
  attitude_ekf.q[0] = cr * cp;
  attitude_ekf.q[1] = sr * cp;
  attitude_ekf.q[2] = cr * sp;
  attitude_ekf.q[3] = -sr * sp;
  /* Retain the last measured heading, never a target-derived heading. */
  float cy=cosf(0.5f*retained_yaw),sy=sinf(0.5f*retained_yaw);
  float q0=attitude_ekf.q[0],q1=attitude_ekf.q[1],q2=attitude_ekf.q[2],q3=attitude_ekf.q[3];
  attitude_ekf.q[0]=cy*q0-sy*q3;attitude_ekf.q[1]=cy*q1-sy*q2;
  attitude_ekf.q[2]=cy*q2+sy*q1;attitude_ekf.q[3]=cy*q3+sy*q0;
  QuaternionNormalize(attitude_ekf.q);

  for (uint32_t i = 0; i < 3U; i++)
  {
    attitude_ekf.gyro_bias_rad_s[i] = gyro_bias_rad_s[i];
    attitude_ekf.covariance[i][i] = 0.0076f;      /* (5 deg)^2 */
    attitude_ekf.covariance[i + 3U][i + 3U] = 0.000076f; /* (0.5 dps)^2 */
  }

  QuaternionToEuler(
      attitude_ekf.q,
      &attitude_ekf.roll_deg,
      &attitude_ekf.pitch_deg,
      &attitude_ekf.yaw_deg);
  attitude_ekf.previous_yaw_deg = attitude_ekf.yaw_deg;
}

static void AttitudeEKF_PropagateCovariance(
    const float angular_rate_rad_s[3],
    float dt)
{
  float transition[6][6] = {0};
  float intermediate[6][6] = {0};
  float propagated[6][6] = {0};
  float wx = angular_rate_rad_s[0];
  float wy = angular_rate_rad_s[1];
  float wz = angular_rate_rad_s[2];

  for (uint32_t i = 0; i < 6U; i++)
  {
    transition[i][i] = 1.0f;
  }

  /* Error state: d(theta)/dt = -cross(w)*theta - gyro_bias_error. */
  transition[0][1] =  wz * dt;
  transition[0][2] = -wy * dt;
  transition[1][0] = -wz * dt;
  transition[1][2] =  wx * dt;
  transition[2][0] =  wy * dt;
  transition[2][1] = -wx * dt;
  transition[0][3] = -dt;
  transition[1][4] = -dt;
  transition[2][5] = -dt;

  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 6U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        intermediate[row][column] +=
            transition[row][k] * attitude_ekf.covariance[k][column];
      }
    }
  }

  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 6U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        propagated[row][column] +=
            intermediate[row][k] * transition[column][k];
      }
    }
  }

  float gyro_variance = EKF_GYRO_NOISE_RAD_S * EKF_GYRO_NOISE_RAD_S * dt;
  float bias_variance = EKF_BIAS_WALK_RAD_S2 * EKF_BIAS_WALK_RAD_S2 * dt;
  for (uint32_t i = 0; i < 3U; i++)
  {
    propagated[i][i] += gyro_variance;
    propagated[i + 3U][i + 3U] += bias_variance;
  }

  memcpy(attitude_ekf.covariance, propagated, sizeof(propagated));
}

static void AttitudeEKF_AccelerometerUpdate(float ax, float ay, float az)
{
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  float norm_g = norm / ACCEL_LSB_PER_G;

  /* Acceleration magnitude and direction gates prevent a linear acceleration
     from being interpreted as gravity and pulling the displayed attitude. */
  if ((norm < 1.0f)
      || (norm_g < ACCEL_GATE_MIN_G)
      || (norm_g > ACCEL_GATE_MAX_G))
  {
    attitude_ekf.accel_rejected++;
    return;
  }

  float measurement[3] = {ax / norm, ay / norm, az / norm};
  const float *q = attitude_ekf.q;
  float predicted[3] =
  {
    2.0f * (q[1] * q[3] - q[0] * q[2]),
    2.0f * (q[0] * q[1] + q[2] * q[3]),
    q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
  };
  float innovation[3] =
  {
    measurement[0] - predicted[0],
    measurement[1] - predicted[1],
    measurement[2] - predicted[2]
  };
  float direction_dot = measurement[0] * predicted[0]
                      + measurement[1] * predicted[1]
                      + measurement[2] * predicted[2];
  if (direction_dot < ACCEL_INNOVATION_MIN_DOT && !attitude_ekf.stationary)
  {
    attitude_ekf.accel_rejected++;
    return;
  }

  /* Jacobian H = [cross(predicted_gravity), 0]. */
  float observation[3][6] = {0};
  observation[0][1] = -predicted[2];
  observation[0][2] =  predicted[1];
  observation[1][0] =  predicted[2];
  observation[1][2] = -predicted[0];
  observation[2][0] = -predicted[1];
  observation[2][1] =  predicted[0];

  float covariance_times_h_transpose[6][3] = {0};
  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        covariance_times_h_transpose[row][column] +=
            attitude_ekf.covariance[row][k] * observation[column][k];
      }
    }
  }

  float innovation_covariance[3][3] = {0};
  float acceleration_error = norm_g - 1.0f;
  float measurement_variance = EKF_ACCEL_NOISE * EKF_ACCEL_NOISE
                             + 0.5f * acceleration_error * acceleration_error;
  /* During motion, a gravity-direction residual can be external acceleration
     (including vibration/lever-arm force), even if |a| remains near1g.
     Robust Cauchy weighting:30mg design scale, at most25x nominal variance.
     Quiet confirmed samples regain nominal weight and allow bounded recovery. */
  if(!attitude_ekf.stationary) {
    float energy=innovation[0]*innovation[0]+innovation[1]*innovation[1]+innovation[2]*innovation[2];
    float scale=1.0f+fminf(24.0f,energy/(ACCEL_DYNAMIC_RESIDUAL_G*ACCEL_DYNAMIC_RESIDUAL_G));
    measurement_variance*=scale;
  }
  attitude_ekf.accel_observation_variance=measurement_variance;
  for (uint32_t row = 0; row < 3U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        innovation_covariance[row][column] +=
            observation[row][k] * covariance_times_h_transpose[k][column];
      }
    }
    innovation_covariance[row][row] += measurement_variance;
  }

  float inverse_innovation_covariance[3][3];
  if (!Matrix3Inverse(innovation_covariance, inverse_innovation_covariance))
  {
    return;
  }

  float gain[6][3] = {0};
  float correction[6] = {0};
  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 3U; k++)
      {
        gain[row][column] +=
            covariance_times_h_transpose[row][k]
          * inverse_innovation_covariance[k][column];
      }
      correction[row] += gain[row][column] * innovation[column];
    }
  }

  /* Constrain the ACTUAL gain, not only the injected correction. Gravity
     cannot observe heading. Accel alone is not sufficient evidence for gyro
     bias adaptation in a moving bench rig. Keep its bias gain zero, then use
     Joseph covariance with this same constrained gain. Mag observes bias. */
  memset(correction,0,sizeof(correction));
  for(unsigned column=0;column<3;column++) {
    float heading_component=0;
    for(unsigned row=0;row<3;row++)heading_component+=predicted[row]*gain[row][column];
    for(unsigned row=0;row<3;row++)gain[row][column]-=predicted[row]*heading_component;
    for(unsigned row=3;row<6;row++)gain[row][column]=0;
  }
  for(unsigned row=0;row<6;row++)for(unsigned column=0;column<3;column++)
    correction[row]+=gain[row][column]*innovation[column];

  float correction_norm=sqrtf(correction[0]*correction[0]+correction[1]*correction[1]+correction[2]*correction[2]);
  if(correction_norm>ACCEL_MAX_CORRECTION_RAD) {
    float fraction=ACCEL_MAX_CORRECTION_RAD/correction_norm;
    for(unsigned row=0;row<6;row++) {
      correction[row]*=fraction;
      for(unsigned column=0;column<3;column++)gain[row][column]*=fraction;
    }
  }

  /* Right-multiplicative small-angle correction q <- q * dq. */
  float old_q[4] = {q[0], q[1], q[2], q[3]};
  float half_x = 0.5f * correction[0];
  float half_y = 0.5f * correction[1];
  float half_z = 0.5f * correction[2];
  attitude_ekf.q[0] = old_q[0] - old_q[1] * half_x - old_q[2] * half_y - old_q[3] * half_z;
  attitude_ekf.q[1] = old_q[1] + old_q[0] * half_x + old_q[2] * half_z - old_q[3] * half_y;
  attitude_ekf.q[2] = old_q[2] + old_q[0] * half_y - old_q[1] * half_z + old_q[3] * half_x;
  attitude_ekf.q[3] = old_q[3] + old_q[0] * half_z + old_q[1] * half_y - old_q[2] * half_x;
  QuaternionNormalize(attitude_ekf.q);

  attitude_ekf.accel_accepted++;

  float residual_map[6][6]={{0}},ap[6][6]={{0}},updated_covariance[6][6]={{0}};
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++) {
    residual_map[i][j]=(i==j?1.0f:0.0f);
    for(unsigned k=0;k<3;k++)residual_map[i][j]-=gain[i][k]*observation[k][j];
  }
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++)for(unsigned k=0;k<6;k++)
    ap[i][j]+=residual_map[i][k]*attitude_ekf.covariance[k][j];
  for(unsigned i=0;i<6;i++)for(unsigned j=0;j<6;j++) {
    for(unsigned k=0;k<6;k++)updated_covariance[i][j]+=ap[i][k]*residual_map[j][k];
    for(unsigned k=0;k<3;k++)updated_covariance[i][j]+=measurement_variance*gain[i][k]*gain[j][k];
  }

  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 6U; column++)
    {
      attitude_ekf.covariance[row][column] =
          0.5f * (updated_covariance[row][column]
                + updated_covariance[column][row]);
    }
    if (attitude_ekf.covariance[row][row] < 1.0e-9f)
    {
      attitude_ekf.covariance[row][row] = 1.0e-9f;
    }
  }
  ADCS_CovarianceReset(attitude_ekf.covariance,correction);
}

static void __attribute__((unused)) AttitudeEKF_VectorUpdate(
    const float measurement[3],
    const float predicted[3],
    float measurement_variance)
{
  float innovation[3] =
  {
    measurement[0] - predicted[0],
    measurement[1] - predicted[1],
    measurement[2] - predicted[2]
  };

  float observation[3][6] = {0};
  observation[0][1] = -predicted[2];
  observation[0][2] =  predicted[1];
  observation[1][0] =  predicted[2];
  observation[1][2] = -predicted[0];
  observation[2][0] = -predicted[1];
  observation[2][1] =  predicted[0];

  float covariance_times_h_transpose[6][3] = {0};
  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        covariance_times_h_transpose[row][column] +=
            attitude_ekf.covariance[row][k] * observation[column][k];
      }
    }
  }

  float innovation_covariance[3][3] = {0};
  for (uint32_t row = 0; row < 3U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 6U; k++)
      {
        innovation_covariance[row][column] +=
            observation[row][k] * covariance_times_h_transpose[k][column];
      }
    }
    innovation_covariance[row][row] += measurement_variance;
  }

  float inverse_innovation_covariance[3][3];
  if (!Matrix3Inverse(innovation_covariance, inverse_innovation_covariance))
  {
    return;
  }

  float gain[6][3] = {0};
  float correction[6] = {0};
  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 3U; column++)
    {
      for (uint32_t k = 0; k < 3U; k++)
      {
        gain[row][column] +=
            covariance_times_h_transpose[row][k]
          * inverse_innovation_covariance[k][column];
      }
      correction[row] += gain[row][column] * innovation[column];
    }
  }

  float *q = attitude_ekf.q;
  float old_q[4] = {q[0], q[1], q[2], q[3]};
  float half_x = 0.5f * correction[0];
  float half_y = 0.5f * correction[1];
  float half_z = 0.5f * correction[2];
  q[0] = old_q[0] - old_q[1] * half_x - old_q[2] * half_y - old_q[3] * half_z;
  q[1] = old_q[1] + old_q[0] * half_x + old_q[2] * half_z - old_q[3] * half_y;
  q[2] = old_q[2] + old_q[0] * half_y - old_q[1] * half_z + old_q[3] * half_x;
  q[3] = old_q[3] + old_q[0] * half_z + old_q[1] * half_y - old_q[2] * half_x;
  QuaternionNormalize(q);

  for (uint32_t i = 0; i < 3U; i++)
  {
    attitude_ekf.gyro_bias_rad_s[i] += correction[i + 3U];
  }

  float updated_covariance[6][6] = {0};
  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 6U; column++)
    {
      updated_covariance[row][column] = attitude_ekf.covariance[row][column];
      for (uint32_t k = 0; k < 3U; k++)
      {
        updated_covariance[row][column] -=
            gain[row][k] * covariance_times_h_transpose[column][k];
      }
    }
  }

  for (uint32_t row = 0; row < 6U; row++)
  {
    for (uint32_t column = 0; column < 6U; column++)
    {
      attitude_ekf.covariance[row][column] =
          0.5f * (updated_covariance[row][column]
                + updated_covariance[column][row]);
    }
    if (attitude_ekf.covariance[row][row] < 1.0e-9f)
    {
      attitude_ekf.covariance[row][row] = 1.0e-9f;
    }
  }
}

static void RotateBodyToWorld(
    const float q[4],
    const float body[3],
    float world[3])
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  world[0] = (1.0f - 2.0f * (y * y + z * z)) * body[0]
           + 2.0f * (x * y - w * z) * body[1]
           + 2.0f * (x * z + w * y) * body[2];
  world[1] = 2.0f * (x * y + w * z) * body[0]
           + (1.0f - 2.0f * (x * x + z * z)) * body[1]
           + 2.0f * (y * z - w * x) * body[2];
  world[2] = 2.0f * (x * z - w * y) * body[0]
           + 2.0f * (y * z + w * x) * body[1]
           + (1.0f - 2.0f * (x * x + y * y)) * body[2];
}

static void RotateWorldToBody(
    const float q[4],
    const float world[3],
    float body[3])
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  body[0] = (1.0f - 2.0f * (y * y + z * z)) * world[0]
          + 2.0f * (x * y + w * z) * world[1]
          + 2.0f * (x * z - w * y) * world[2];
  body[1] = 2.0f * (x * y - w * z) * world[0]
          + (1.0f - 2.0f * (x * x + z * z)) * world[1]
          + 2.0f * (y * z + w * x) * world[2];
  body[2] = 2.0f * (x * z + w * y) * world[0]
          + 2.0f * (y * z - w * x) * world[1]
          + (1.0f - 2.0f * (x * x + y * y)) * world[2];
}

static HAL_StatusTypeDef AttitudeEKF_SetMagneticReference(void)
{
  float sum[3] = {0.0f, 0.0f, 0.0f};
  uint32_t valid_samples = 0U;

  for (uint32_t attempt = 0; attempt < 80U; attempt++)
  {
    if(emergency_stop)return HAL_ERROR;
    int16_t mx, my, mz;
    if (GY511_MagReadRaw(&mx, &my, &mz) == HAL_OK)
    {
      float sample[3];
      GY511_MagToBody(mx, my, mz, sample);
      for (uint32_t axis = 0; axis < 3U; axis++)
      {
        sum[axis] += sample[axis];
      }
      valid_samples++;
      if (valid_samples >= 32U)
      {
        break;
      }
    }
    App_Wait(14);
  }

  if (valid_samples < 20U)
  {
    return HAL_ERROR;
  }

  for (uint32_t axis = 0; axis < 3U; axis++)
  {
    magnetometer.body[axis] = sum[axis] / (float)valid_samples;
  }
  float norm = sqrtf(
      magnetometer.body[0] * magnetometer.body[0]
    + magnetometer.body[1] * magnetometer.body[1]
    + magnetometer.body[2] * magnetometer.body[2]);
  if (norm < 1.0f)
  {
    return HAL_ERROR;
  }

  float normalized_body[3] =
  {
    magnetometer.body[0] / norm,
    magnetometer.body[1] / norm,
    magnetometer.body[2] / norm
  };
  RotateBodyToWorld(
      attitude_ekf.q,
      normalized_body,
      magnetometer.reference_world);
  magnetometer.reference_norm = norm;
  magnetometer.reference_valid = 1U;
  return HAL_OK;
}

/* Used only if boot had no compass reference. Establish it from a sustained
   quiet field in the existing world frame; never change q or the setpoint. */
static void AttitudeEKF_ObserveInitialMagReference(const float body_mag[3])
{
  static float world_sum[3],norm_sum;
  static unsigned samples;
  static uint32_t epoch;
  if(epoch!=mag_reference_epoch){samples=0U;epoch=mag_reference_epoch;}
  for(unsigned i=0;i<3;i++)if(motor_command[i]>20 || motor_command[i]<-20 || wheel_encoder[i].rpm>120.0f){samples=0U;return;}
  float norm=sqrtf(body_mag[0]*body_mag[0]+body_mag[1]*body_mag[1]+body_mag[2]*body_mag[2]);
  if(!isfinite(norm) || norm<1.0f){samples=0U;return;}
  float normalized[3]={body_mag[0]/norm,body_mag[1]/norm,body_mag[2]/norm},world[3];
  RotateBodyToWorld(attitude_ekf.q,normalized,world);
  if(!samples){memset(world_sum,0,sizeof(world_sum));norm_sum=0;}
  else {
    float dot=0;for(unsigned i=0;i<3;i++)dot+=world[i]*world_sum[i]/(float)samples;
    float ratio=norm/(norm_sum/(float)samples);
    if(dot<0.98f || ratio<MAG_FIELD_MIN_RATIO || ratio>MAG_FIELD_MAX_RATIO){samples=0U;return;}
  }
  for(unsigned i=0;i<3;i++)world_sum[i]+=world[i];
  norm_sum+=norm;samples++;
  if(samples>=32U) {
    float sum_norm=sqrtf(world_sum[0]*world_sum[0]+world_sum[1]*world_sum[1]+world_sum[2]*world_sum[2]);
    for(unsigned i=0;i<3;i++)magnetometer.reference_world[i]=world_sum[i]/sum_norm;
    magnetometer.reference_norm=norm_sum/(float)samples;magnetometer.reference_valid=1U;samples=0U;
  }
}

static void AttitudeEKF_MagnetometerUpdate(const float body_mag[3])
{
  if (magnetometer.reference_valid == 0U)
  {
    AttitudeEKF_ObserveInitialMagReference(body_mag);
    clean_heading_samples=0U;magnetic_heading_quality=0U;
    return;
  }

  float norm = sqrtf(
      body_mag[0] * body_mag[0]
    + body_mag[1] * body_mag[1]
    + body_mag[2] * body_mag[2]);
  float field_ratio = norm / magnetometer.reference_norm;
  mag_field_ratio=field_ratio;
  mag_innovation_deg=0.0f;
  if ((norm < 1.0f)
      || (field_ratio < MAG_FIELD_MIN_RATIO)
      || (field_ratio > MAG_FIELD_MAX_RATIO))
  {
    mag_reject_reason=1U;
    magnetometer.rejected_samples++;
    attitude_ekf.mag_rejected++;
    clean_heading_samples=0U;magnetic_heading_quality=0U;
    return;
  }

  float measurement[3] =
  {
    body_mag[0] / norm,
    body_mag[1] / norm,
    body_mag[2] / norm
  };
  float predicted[3];
  RotateWorldToBody(
      attitude_ekf.q,
      magnetometer.reference_world,
      predicted);

  /* The magnetometer observes heading only.  Project both vectors onto the
     plane perpendicular to gravity so motor/soft-iron errors cannot pull
     roll and pitch back toward the calibration pose. */
  const float *q = attitude_ekf.q;
  float gravity_body[3] =
  {
    2.0f * (q[1] * q[3] - q[0] * q[2]),
    2.0f * (q[0] * q[1] + q[2] * q[3]),
    q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
  };
  float measurement_horizontal[3];
  float predicted_horizontal[3];
  float measurement_vertical = measurement[0] * gravity_body[0]
                             + measurement[1] * gravity_body[1]
                             + measurement[2] * gravity_body[2];
  float predicted_vertical = predicted[0] * gravity_body[0]
                           + predicted[1] * gravity_body[1]
                           + predicted[2] * gravity_body[2];
  float measurement_horizontal_norm = 0.0f;
  float predicted_horizontal_norm = 0.0f;
  for (uint32_t i = 0U; i < 3U; i++)
  {
    measurement_horizontal[i] = measurement[i]
                              - measurement_vertical * gravity_body[i];
    predicted_horizontal[i] = predicted[i]
                            - predicted_vertical * gravity_body[i];
    measurement_horizontal_norm +=
        measurement_horizontal[i] * measurement_horizontal[i];
    predicted_horizontal_norm +=
        predicted_horizontal[i] * predicted_horizontal[i];
  }
  measurement_horizontal_norm = sqrtf(measurement_horizontal_norm);
  predicted_horizontal_norm = sqrtf(predicted_horizontal_norm);
  if ((measurement_horizontal_norm < 0.15f)
      || (predicted_horizontal_norm < 0.15f))
  {
    mag_reject_reason=2U;
    magnetometer.rejected_samples++;
    attitude_ekf.mag_rejected++;
    clean_heading_samples=0U;magnetic_heading_quality=0U;
    return;
  }
  for (uint32_t i = 0U; i < 3U; i++)
  {
    measurement_horizontal[i] /= measurement_horizontal_norm;
    predicted_horizontal[i] /= predicted_horizontal_norm;
  }

  float heading_dot = ClampFloat(
      measurement_horizontal[0] * predicted_horizontal[0]
    + measurement_horizontal[1] * predicted_horizontal[1]
    + measurement_horizontal[2] * predicted_horizontal[2],
      -1.0f,
      1.0f);
  float cross[3] =
  {
    measurement_horizontal[1] * predicted_horizontal[2]
      - measurement_horizontal[2] * predicted_horizontal[1],
    measurement_horizontal[2] * predicted_horizontal[0]
      - measurement_horizontal[0] * predicted_horizontal[2],
    measurement_horizontal[0] * predicted_horizontal[1]
      - measurement_horizontal[1] * predicted_horizontal[0]
  };
  float heading_error_rad = atan2f(
      gravity_body[0] * cross[0]
    + gravity_body[1] * cross[1]
    + gravity_body[2] * cross[2],
      heading_dot);
  mag_innovation_deg=heading_error_rad*RAD_TO_DEG;
  if (heading_dot < MAG_INNOVATION_MIN_DOT)
  {
    mag_reject_reason=3U;magnetometer.rejected_samples++;attitude_ekf.mag_rejected++;
    clean_heading_samples=0U;magnetic_heading_quality=0U;return;
  }

  float correction[3],bias_correction[3];
  float motor_activity=0.0f;
  uint8_t quiet=1U;
  for(unsigned i=0;i<3;i++)
  {
    motor_activity=fmaxf(motor_activity,fabsf((float)motor_command[i])/255.0f);
    if(motor_command[i]>20 || motor_command[i]<-20 || wheel_encoder[i].rpm>120.0f)quiet=0U;
  }
  /* Downweight heading while brush motors generate magnetic interference.
     Bias learning requires a quiet motor/field environment. */
  if(quiet && fabsf(field_ratio-1.0f)<0.10f && fabsf(heading_error_rad)<0.26f)
  {
    if(clean_heading_samples<25U)clean_heading_samples++;
  }
  else clean_heading_samples=0U;
  uint8_t stable_clean=(clean_heading_samples>=25U)?1U:0U;
  float noise=Heading_NoiseVariance(field_ratio,motor_activity,stable_clean);
  if(!Heading_Update(attitude_ekf.covariance,gravity_body,heading_error_rad,
                     noise,quiet && stable_clean && attitude_ekf.stationary,correction,bias_correction))
  {
    mag_reject_reason=4U;attitude_ekf.mag_rejected++;clean_heading_samples=0U;magnetic_heading_quality=0U;return;
  }

  float old_q[4] = {q[0], q[1], q[2], q[3]};
  float half_x = 0.5f * correction[0];
  float half_y = 0.5f * correction[1];
  float half_z = 0.5f * correction[2];
  attitude_ekf.q[0] = old_q[0] - old_q[1] * half_x
                    - old_q[2] * half_y - old_q[3] * half_z;
  attitude_ekf.q[1] = old_q[1] + old_q[0] * half_x
                    + old_q[2] * half_z - old_q[3] * half_y;
  attitude_ekf.q[2] = old_q[2] + old_q[0] * half_y
                    - old_q[1] * half_z + old_q[3] * half_x;
  attitude_ekf.q[3] = old_q[3] + old_q[0] * half_z
                    + old_q[1] * half_y - old_q[2] * half_x;
  QuaternionNormalize(attitude_ekf.q);

  for(unsigned i=0;i<3;i++)
  {
    attitude_ekf.gyro_bias_rad_s[i]+=bias_correction[i];
  }
  last_heading_update_ms=HAL_GetTick();
  magnetic_heading_quality=stable_clean?2U:1U;
  mag_reject_reason=0U;
  attitude_ekf.mag_accepted++;
}

static void AttitudeEKF_RefreshEuler(void)
{
  QuaternionToEuler(
      attitude_ekf.q,
      &attitude_ekf.roll_deg,
      &attitude_ekf.pitch_deg,
      &attitude_ekf.yaw_deg);
  attitude_ekf.yaw_deg = WrapAngle180(attitude_ekf.yaw_deg);
  attitude_ekf.previous_yaw_deg = attitude_ekf.yaw_deg;
}

static void Magnetometer_ProcessSample(HAL_StatusTypeDef status,int16_t mx,int16_t my,int16_t mz)
{
  if(status==HAL_OK) {
    last_mag_sample_ms=HAL_GetTick();mag_status=HAL_OK;mag_read_failures=0U;
    GY511_MagToBody(mx,my,mz,magnetometer.body);
    AttitudeEKF_MagnetometerUpdate(magnetometer.body);AttitudeEKF_RefreshEuler();
  } else if(status!=HAL_BUSY) {
    mag_io_errors++;magnetic_heading_quality=0U;clean_heading_samples=0U;
    if(mag_reject_reason!=5U)mag_reject_reason=7U;
    if(mag_read_failures<10U)mag_read_failures++;
    if(mag_read_failures>=10U)mag_status=HAL_ERROR;
    /* Optional sensor only: do not modify fusion/PID/target/quaternion. */
  } else if(HAL_GetTick()-last_mag_sample_ms>200U) {
    mag_reject_reason=6U;magnetic_heading_quality=0U;
  }
}

static void AttitudeEKF_Update(
    int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz,
    float dt)
{
  float angular_rate_rad_s[3];
  float body_accel_raw[3];
  SensorToBodyGyroRad((float)gx, (float)gy, (float)gz, angular_rate_rad_s);
  SensorToBodyAccel((float)ax, (float)ay, (float)az, body_accel_raw);

  if(gx>=32700 || gx<=-32700 || gy>=32700 || gy<=-32700 || gz>=32700 || gz<=-32700)
    if(attitude_ekf.gyro_clipped_samples<UINT32_MAX)attitude_ekf.gyro_clipped_samples++;
  if(ax>=32700 || ax<=-32700 || ay>=32700 || ay<=-32700 || az>=32700 || az<=-32700)
    if(attitude_ekf.accel_clipped_samples<UINT32_MAX)attitude_ekf.accel_clipped_samples++;

  attitude_ekf.last_dt_us = (uint32_t)(dt * 1000000.0f + 0.5f);
  float accel_norm_g = sqrtf(
      body_accel_raw[0] * body_accel_raw[0]
    + body_accel_raw[1] * body_accel_raw[1]
    + body_accel_raw[2] * body_accel_raw[2]) / ACCEL_LSB_PER_G;
  float corrected_gyro_magnitude_sq = 0.0f;
  for (uint32_t i = 0U; i < 3U; i++)
  {
    float corrected = angular_rate_rad_s[i] - attitude_ekf.gyro_bias_rad_s[i];
    corrected_gyro_magnitude_sq += corrected * corrected;
  }
  float corrected_gyro_dps = sqrtf(corrected_gyro_magnitude_sq) * RAD_TO_DEG;
  float accel_change_sq=0;
  for(unsigned i=0;i<3;i++) {
    float change=body_accel_raw[i]-attitude_ekf.previous_accel_raw[i];
    accel_change_sq+=change*change;
    attitude_ekf.previous_accel_raw[i]=body_accel_raw[i];
  }
  uint8_t stable_accel=!attitude_ekf.accel_history_valid ||
      accel_change_sq<(.02f*ACCEL_LSB_PER_G)*(.02f*ACCEL_LSB_PER_G);
  attitude_ekf.accel_history_valid=1U;
  uint32_t now_ms = HAL_GetTick();
  if ((fabsf(accel_norm_g - 1.0f) <= STATIONARY_ACCEL_TOL_G)
      && stable_accel && (corrected_gyro_dps <= STATIONARY_GYRO_DPS))
  {
    if (attitude_ekf.stationary_since_ms == 0U)
    {
      attitude_ekf.stationary_since_ms = now_ms;
    }
    if ((now_ms - attitude_ekf.stationary_since_ms) >= STATIONARY_CONFIRM_MS)
    {
      attitude_ekf.stationary = 1U;
      /* A low gyro rate does not prove zero yaw motion. Estimate gyro
         bias from observable accel/magnetic residuals, not from this flag. */
    }
  }
  else
  {
    attitude_ekf.stationary_since_ms = 0U;
    attitude_ekf.stationary = 0U;
  }

  for (uint32_t i = 0; i < 3U; i++)
  {
    angular_rate_rad_s[i] -= attitude_ekf.gyro_bias_rad_s[i];
  }

  float *q = attitude_ekf.q;
  ADCS_IntegrateBodyRate(q,angular_rate_rad_s,dt);
  QuaternionNormalize(q);

  AttitudeEKF_PropagateCovariance(angular_rate_rad_s, dt);
  AttitudeEKF_AccelerometerUpdate(
      body_accel_raw[0],
      body_accel_raw[1],
      body_accel_raw[2]);

  float new_yaw_deg;
  QuaternionToEuler(
      attitude_ekf.q,
      &attitude_ekf.roll_deg,
      &attitude_ekf.pitch_deg,
      &new_yaw_deg);
  new_yaw_deg = WrapAngle180(new_yaw_deg);

  float instantaneous_yaw_rate =
      WrapAngle180(new_yaw_deg - attitude_ekf.previous_yaw_deg) / dt;
  attitude_ekf.yaw_rate_dps =
      0.80f * attitude_ekf.yaw_rate_dps + 0.20f * instantaneous_yaw_rate;
  attitude_ekf.previous_yaw_deg = new_yaw_deg;
  attitude_ekf.yaw_deg = new_yaw_deg;
}

static void AttitudeControl_Reset(void)
{
  /* Controller state follows the estimator; it never overwrites estimator
     output.  Measured attitude and target attitude are separate states. */
  roll_control.angle_deg = attitude_ekf.roll_deg;
  if(!control_targets_initialized)roll_control.setpoint_deg = attitude_ekf.roll_deg;
  roll_control.integral = 0.0f;
  roll_control.rate_dps = 0.0f;
  roll_control.error_deg = 0.0f;
  roll_control.p_term = 0.0f;
  roll_control.i_term = 0.0f;
  roll_control.d_term = 0.0f;
  roll_control.output_pwm = 0.0f;
  roll_control.saturated = 0U;

  pitch_control.angle_deg = attitude_ekf.pitch_deg;
  if(!control_targets_initialized)pitch_control.setpoint_deg = attitude_ekf.pitch_deg;
  pitch_control.integral = 0.0f;
  pitch_control.rate_dps = 0.0f;
  pitch_control.error_deg = 0.0f;
  pitch_control.p_term = 0.0f;
  pitch_control.i_term = 0.0f;
  pitch_control.d_term = 0.0f;
  pitch_control.output_pwm = 0.0f;
  pitch_control.saturated = 0U;

  yaw_control.angle_deg = attitude_ekf.yaw_deg;
  if(!control_targets_initialized)yaw_control.setpoint_deg = attitude_ekf.yaw_deg;
  yaw_control.integral = 0.0f;
  yaw_control.rate_dps = 0.0f;
  yaw_control.error_deg = 0.0f;
  yaw_control.p_term = 0.0f;
  yaw_control.i_term = 0.0f;
  yaw_control.d_term = 0.0f;
  yaw_control.output_pwm = 0.0f;
  yaw_control.saturated = 0U;
  for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
  {
    wheel_speed_control[axis].integral = 0.0f;
    wheel_speed_control[axis].previous_feedback_rpm = 0.0f;
    wheel_speed_control[axis].output_pwm = 0.0f;
  }
  control_targets_initialized=1U;
  Motors_StopAll();
  /* Keep the operator PID request across calibration, faults and recovery. */
}

static void AttitudeControl_HoldCurrentYaw(void)
{
  yaw_control.setpoint_deg=attitude_ekf.yaw_deg;
  yaw_control.integral=0.0f;
  control_targets_initialized=1U;
}

static int32_t AxisControl_Update(
    AxisControl_t *control,
    float fused_angle_deg,
    float body_error_deg,
    float axis_rate_dps,
    float kp,
    float ki,
    float kd,
    float integral_limit,
    float motor_sign,
    float dt,
    uint8_t enabled)
{
  control->rate_dps =
      0.80f * control->rate_dps + 0.20f * axis_rate_dps;
  control->angle_deg = WrapAngle180(fused_angle_deg);
  control->error_deg = body_error_deg;

  if (enabled == 0U)
  {
    control->integral = 0.0f;
    control->p_term = 0.0f;
    control->i_term = 0.0f;
    control->d_term = 0.0f;
    control->output_pwm = 0.0f;
    control->saturated = 0U;
    return 0;
  }

  float candidate_integral = ClampFloat(
      control->integral + control->error_deg * dt,
      -integral_limit,
      integral_limit);

  float p_term = kp * control->error_deg;
  float i_term = ki * candidate_integral;
  float d_term = -kd * control->rate_dps;
  float output = p_term + i_term + d_term;

  /* Conditional integration prevents wind-up at the PWM limits. */
  if ((fabsf(output) < (float)MOTOR_MAX_PWM) ||
      ((output > 0.0f) && (control->error_deg < 0.0f)) ||
      ((output < 0.0f) && (control->error_deg > 0.0f)))
  {
    control->integral = candidate_integral;
  }

  i_term = ki * control->integral;
  output = p_term + i_term + d_term;
  control->saturated = (fabsf(output) > (float)MOTOR_MAX_PWM) ? 1U : 0U;

  output = ClampFloat(output, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
  control->p_term = motor_sign * p_term;
  control->i_term = motor_sign * i_term;
  control->d_term = motor_sign * d_term;
  control->output_pwm = motor_sign * output;
  return (int32_t)control->output_pwm;
}

static int32_t WheelSpeedControl_Update(
    MotorAxis_t axis,
    float dt)
{
  WheelSpeedControl_t *control = &wheel_speed_control[axis];
  WheelEncoder_t *encoder = &wheel_encoder[axis];

  if (fabsf(control->setpoint_rpm) > 1.0f)
  {
    encoder->assumed_direction = (control->setpoint_rpm > 0.0f) ? 1 : -1;
  }
  control->feedback_rpm =
      (float)encoder->assumed_direction * encoder->rpm;
  control->error_rpm = control->setpoint_rpm - control->feedback_rpm;

  if ((control->gains_configured == 0U)
      || (encoder->speed_valid == 0U)
      || (encoder->fault_flags != 0U))
  {
    control->integral = 0.0f;
    control->p_term = 0.0f;
    control->i_term = 0.0f;
    control->d_term = 0.0f;
    control->output_pwm = 0.0f;
    control->saturated = 0U;
    control->previous_feedback_rpm = control->feedback_rpm;
    return 0;
  }

  float derivative_feedback =
      (control->feedback_rpm - control->previous_feedback_rpm) / dt;
  float integral_limit = (control->ki > 1.0e-6f)
      ? ((float)MOTOR_MAX_PWM / control->ki) : 0.0f;
  float candidate_integral = (integral_limit > 0.0f)
      ? ClampFloat(control->integral + control->error_rpm * dt,
                   -integral_limit,
                   integral_limit)
      : 0.0f;
  float p_term = control->kp * control->error_rpm;
  float i_term = control->ki * candidate_integral;
  float d_term = -control->kd * derivative_feedback;
  float output = p_term + i_term + d_term;

  if ((fabsf(output) < (float)MOTOR_MAX_PWM)
      || ((output > 0.0f) && (control->error_rpm < 0.0f))
      || ((output < 0.0f) && (control->error_rpm > 0.0f)))
  {
    control->integral = candidate_integral;
  }

  control->p_term = p_term;
  control->i_term = control->ki * control->integral;
  control->d_term = d_term;
  output = control->p_term + control->i_term + control->d_term;
  control->saturated = (fabsf(output) > (float)MOTOR_MAX_PWM) ? 1U : 0U;
  control->output_pwm = ClampFloat(
      output, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
  control->previous_feedback_rpm = control->feedback_rpm;
  return (int32_t)control->output_pwm;
}

static HAL_StatusTypeDef Fusion_RestartAfterStationaryCalibration(
    float gyro_bias_rad_s[3],
    float accel_average_raw[3])
{
  Motors_StopAll();
  /* Keep the operator PID request across calibration, faults and recovery. */
  if (Sensor_Calibrate(gyro_bias_rad_s, accel_average_raw) != HAL_OK)
  {
    return HAL_ERROR;
  }

  AttitudeEKF_Init(gyro_bias_rad_s, accel_average_raw);
  App_Wait(30);
  if (mag_status!=HAL_OK || (!magnetometer.reference_valid && AttitudeEKF_SetMagneticReference() != HAL_OK))
  {
    static uint8_t error_message[] =
        "CAL WARN TYPE=FUSION REASON=MAG_UNAVAILABLE CONTINUE=GYRO_ACCEL\r\n";
    TelemetryProtocol_Event(error_message, sizeof(error_message) - 1U);
  }
  AttitudeControl_Reset();

  {
    static uint8_t ready_message[] =
        "9DOF EKF READY: YAW=0 AT CURRENT MAGNETIC HEADING\r\n";
    TelemetryProtocol_Event(ready_message, sizeof(ready_message) - 1U);
  }
  return HAL_OK;
}

static HAL_StatusTypeDef Fusion_StartWithStoredCalibration(void)
{
  Motors_StopAll();
  float sum[3]={0};unsigned samples=0;
  for(unsigned attempt=0;attempt<40U && samples<16U;attempt++) {
    int16_t ax,ay,az,t,gx,gy,gz;float body[3];
    if(emergency_stop)return HAL_ERROR;
    if(MPU6050_ReadRaw(&ax,&ay,&az,&t,&gx,&gy,&gz)==HAL_OK) {
      SensorToBodyAccel(ax,ay,az,body);
      float norm=sqrtf(body[0]*body[0]+body[1]*body[1]+body[2]*body[2])/ACCEL_LSB_PER_G;
      if(norm>=ACCEL_GATE_MIN_G && norm<=ACCEL_GATE_MAX_G){for(unsigned i=0;i<3;i++)sum[i]+=body[i];samples++;}
    }
    App_Wait(5);
  }
  if(samples<16U)return HAL_ERROR;
  for(unsigned i=0;i<3;i++)sum[i]/=(float)samples;
  AttitudeEKF_Init(calibration_gyro_bias,sum);
  if(mag_status==HAL_OK && !magnetometer.reference_valid)(void)AttitudeEKF_SetMagneticReference();
  AttitudeControl_Reset();
  const uint8_t ready[]="CAL STATUS TYPE=FUSION PHASE=READY CALIBRATION=STORED\r\n";
  TelemetryProtocol_Event(ready,sizeof(ready)-1U);
  return HAL_OK;
}

static HAL_StatusTypeDef Fusion_CalibrateAndStart(
    float gyro_bias_rad_s[3],
    float accel_average_raw[3])
{
  Motors_StopAll();
  if(calibration_gyro_valid)return Fusion_StartWithStoredCalibration();

  if (magnetometer.calibrated == 0U)
  {
    if (GY511_MagCalibrate() != HAL_OK)
    {
      static uint8_t error_message[] =
          "MAG CAL ERROR: ROTATE FARTHER IN ALL 3 AXES AND CHECK GY-511\r\n";
      TelemetryProtocol_Event(error_message, sizeof(error_message) - 1U);
      return HAL_ERROR;
    }
  }

  {
    static uint8_t still_message[] =
        "CAL STATUS TYPE=GYRO PHASE=PREPARE KEEP_SYSTEM_STILL\r\n";
    TelemetryProtocol_Event(still_message, sizeof(still_message) - 1U);
  }
  App_Wait(2000);

  return Fusion_RestartAfterStationaryCalibration(
      gyro_bias_rad_s,
      accel_average_raw);
}

static void I2C_ScanAndReport(void)
{
  char message[96];
  uint8_t found = 0;

  for (uint8_t address = 0x08; address <= 0x77; address++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 1, 10) == HAL_OK)
    {
      uint8_t who = 0;
      HAL_StatusTypeDef who_status = HAL_I2C_Mem_Read(
          &hi2c1,
          (uint16_t)(address << 1),
          MPU6050_WHO_AM_I,
          I2C_MEMADD_SIZE_8BIT,
          &who,
          1,
          50);

      int length = snprintf(
          message,
          sizeof(message),
          "I2C FOUND ADDR=0x%02X REG75=%s0x%02X\r\n",
          address,
          (who_status == HAL_OK) ? "" : "ERR/",
          who);
      TelemetryProtocol_Event((uint8_t *)message, length);
      App_Wait(30);
      found = 1;
    }
  }

  if (!found)
  {
    int length = snprintf(
        message,
        sizeof(message),
        "I2C SCAN: NO DEVICE, SCL=%u SDA=%u\r\n",
        (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
        (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));
    TelemetryProtocol_Event((uint8_t *)message, length);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Select application IRQ vectors before HAL configures SysTick/UART/DMA,
   * also when entered by the ROM bootloader GO command instead of reset. */
  SCB->VTOR = FLASH_BASE;
  __DSB();
  __ISB();
  __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  float calibrated_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
  float initial_accel_average[3] = {0.0f, 0.0f, ACCEL_LSB_PER_G};

  Motor_Init();
  WheelEncoders_Init();
  Motors_StopAll();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  CommandParser_Init();TelemetryProtocol_Init();
  (void)TelemetryUART_Init();
  App_Wait(100);
  I2C1_BusRecover();
  mpu_status = MPU6050_Init(&mpu_who_am_i);
  mag_status = GY511_MagInit();
  Calibration_Load();
  bmp280_status = BMP280_Init();
  ina219_status = INA219_Init();
  aht30_status = AHT30_Init();

  App_Wait(1500); /* Chờ USB COM */

  {
    static uint8_t encoder_message[] =
        "ENCODERS 1CH ROLL=PA8 PITCH=PC6 YAW=PA0 PPR=25 DIR=UNKNOWN\r\n";
    TelemetryProtocol_Event(encoder_message, sizeof(encoder_message) - 1U);
    App_Wait(20);
  }

  {
    static uint8_t motor_message[] =
        "MOTORS 3AXIS ROLL=PA6,PC4,PC5 PITCH=PA7,PB12,PB13 "
        "YAW=PB0,PB1,PB8 ORDER=PWM,IN1,IN2 PWM_HZ=20500\r\n";
    TelemetryProtocol_Event(motor_message, sizeof(motor_message) - 1U);
    App_Wait(20);
  }

  if (mag_status == HAL_OK)
  {
    char mag_message[96];
    int length = snprintf(
        mag_message,
        sizeof(mag_message),
        "GY511 DETECTED TYPE=%s ADDR=0x%02X WHO=0x%02X\r\n",
        (gy511_mag_type == GY511_MAG_LSM303D) ? "LSM303D" : "LSM303DLHC",
        (unsigned int)(gy511_mag_address >> 1),
        gy511_mag_who_am_i);
    TelemetryProtocol_Event((uint8_t *)mag_message, length);
    App_Wait(20);
  }

  {
    char sensor_message[160];
    int length = snprintf(
        sensor_message,
        sizeof(sensor_message),
        "SLOW SENSORS BMP280=%u ADDR=0x%02X INA219=%u ADDR=0x40 "
        "AHT30=%u ADDR=0x38 CONTROL=200Hz NONBLOCKING=1\r\n",
        (unsigned int)bmp280_status,
        (unsigned int)(bmp280.address >> 1),
        (unsigned int)ina219_status,
        (unsigned int)aht30_status);
    TelemetryProtocol_Event((uint8_t *)sensor_message, length);
    App_Wait(20);
  }

  if (mpu_status == HAL_OK)
  {
    fusion_status = Fusion_CalibrateAndStart(
        calibrated_gyro_bias,
        initial_accel_average);
  }

  uint32_t last_control_tick = HAL_GetTick();
  uint32_t last_mag_tick = HAL_GetTick();
  uint32_t last_encoder_control_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t next_scan_tick = 0;
	  int16_t ax, ay, az;
	  int16_t gx, gy, gz;
	  int16_t temperature;
	  /* The extended debug frame can approach 2 kB. Keep it out of the
	     Cortex-M stack and match the USB CDC transmit buffer capacity. */
	  static char message[512];
	  uint32_t now = HAL_GetTick();
	  UsbCommand_t usb_command = UsbCommand_Poll();

	  if (usb_command != USB_COMMAND_NONE)
	  {
	    uint32_t command_started_ms=HAL_GetTick();
	    if (usb_command == USB_COMMAND_UNKNOWN)
	    {
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "NACK CMD=UNKNOWN RX=%s VALID=CAL_GYRO,CAL_MAG,CAL_ACCEL_START,"
	          "CAL_ACCEL_CAPTURE,CAL_ACCEL_CANCEL,ZERO_YAW,STATUS,SET_ATTITUDE,"
	          "PID,MODE,CAMERA_CAPTURE,ESTOP,CTRL_MODE,SET_WHEEL_RPM,"
	          "WHEEL_PID,RPM_LIMIT,ENCODER_CLEAR\r\n",
	          last_received_command);
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_STATUS)
	    {
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=STATUS ACCEL_ACTIVE=%u ACCEL_CAPTURED=%u/6 "
	          "MAG_CAL=%u FUSION=%u BMP=%u INA=%u AHT=%u MODE=%s "
	          "CTRL=%s PID=%u,%u,%u ESTOP=%u EF=%u,%u,%u "
	          "ELIM=%u,%u,%u WREADY=%u,%u,%u\r\n",
	          accel_calibration.active,
	          accel_calibration.count,
	          magnetometer.calibrated,
	          (unsigned int)fusion_status,
	          (unsigned int)bmp280_status,
	          (unsigned int)ina219_status,
	          (unsigned int)aht30_status,
	          (system_mode == SYSTEM_MODE_CHARGING) ? "CHARGING" : "WORKING",
	          (control_mode == CONTROL_MODE_ATTITUDE)
	              ? "ATTITUDE" : "WHEEL_SPEED_TEST",
	          pid_enabled[MOTOR_AXIS_ROLL],
	          pid_enabled[MOTOR_AXIS_PITCH],
	          pid_enabled[MOTOR_AXIS_YAW],
	          emergency_stop,
	          wheel_encoder[MOTOR_AXIS_ROLL].fault_flags,
	          wheel_encoder[MOTOR_AXIS_PITCH].fault_flags,
	          wheel_encoder[MOTOR_AXIS_YAW].fault_flags,
	          wheel_encoder[MOTOR_AXIS_ROLL].overspeed_limited,
	          wheel_encoder[MOTOR_AXIS_PITCH].overspeed_limited,
	          wheel_encoder[MOTOR_AXIS_YAW].overspeed_limited,
	          wheel_speed_control[MOTOR_AXIS_ROLL].gains_configured,
	          wheel_speed_control[MOTOR_AXIS_PITCH].gains_configured,
	          wheel_speed_control[MOTOR_AXIS_YAW].gains_configured);
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_SET_ATTITUDE)
	    {
	      control_targets_initialized=1U;
	      roll_control.setpoint_deg = WrapAngle180(
	          pending_setpoint_deg[MOTOR_AXIS_ROLL]);
	      pitch_control.setpoint_deg = ClampFloat(
	          pending_setpoint_deg[MOTOR_AXIS_PITCH], -89.0f, 89.0f);
	      yaw_control.setpoint_deg = WrapAngle180(
	          pending_setpoint_deg[MOTOR_AXIS_YAW]);
	      roll_control.integral = 0.0f;
	      pitch_control.integral = 0.0f;
	      yaw_control.integral = 0.0f;
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=SET_ATTITUDE TARGET_mdeg=%ld,%ld,%ld\r\n",
	          (long)(roll_control.setpoint_deg * 1000.0f),
	          (long)(pitch_control.setpoint_deg * 1000.0f),
	          (long)(yaw_control.setpoint_deg * 1000.0f));
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_PID)
	    {
	      uint8_t arm_ok=!(pending_pid_enable && emergency_stop);
	      if(!arm_ok) {
	        const uint8_t error[]="NACK CMD=PID REASON=ESTOP_ACTIVE\r\n";
	        TelemetryProtocol_Event(error,sizeof(error)-1U);
	      } else {
	      if (pending_pid_axis == MOTOR_AXIS_COUNT)
	      {
	        for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	        {
	          pid_enabled[axis] = pending_pid_enable;
	          if (pending_pid_enable == 0U) Motor_SetAxisCommand((MotorAxis_t)axis, 0);
	        }
	      }
	      else
	      {
	        pid_enabled[pending_pid_axis] = pending_pid_enable;
	        if (pending_pid_enable == 0U)
	        {
	          Motor_SetAxisCommand((MotorAxis_t)pending_pid_axis, 0);
	        }
	      }
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=PID ENABLE=%u STATE=%u,%u,%u AUTHORITY=OPERATOR\r\n",
	          pending_pid_enable,
	          pid_enabled[MOTOR_AXIS_ROLL],
	          pid_enabled[MOTOR_AXIS_PITCH],
	          pid_enabled[MOTOR_AXIS_YAW]);
	      TelemetryProtocol_Event((uint8_t *)message, length);
	      }
	    }
	    else if ((usb_command == USB_COMMAND_MODE_WORKING)
	             || (usb_command == USB_COMMAND_MODE_CHARGING))
	    {
	      system_mode = (usb_command == USB_COMMAND_MODE_CHARGING)
	          ? SYSTEM_MODE_CHARGING : SYSTEM_MODE_WORKING;
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=MODE VALUE=%s\r\n",
	          (system_mode == SYSTEM_MODE_CHARGING) ? "CHARGING" : "WORKING");
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_CAMERA_CAPTURE)
	    {
	      static uint8_t camera_message[] =
	          "NACK CMD=CAMERA_CAPTURE REASON=CAMERA_NOT_CONFIGURED\r\n";
	      TelemetryProtocol_Event(camera_message, sizeof(camera_message) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_ESTOP)
	    {
	      emergency_stop = 1U;
	      Motors_StopAll();
      for(unsigned axis=0;axis<MOTOR_AXIS_COUNT;axis++)pid_enabled[axis]=0U;
	      static uint8_t estop_message[] =
	          "ACK CMD=EMERGENCY_STOP ACTIVE=1\r\n";
	      TelemetryProtocol_Event(estop_message, sizeof(estop_message) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_ESTOP_CLEAR)
	    {
	      Motors_StopAll();
	      /* Explicit user action: clearing ESTOP does not re-arm motors. */
	      emergency_stop = 0U;
      for(unsigned axis=0;axis<MOTOR_AXIS_COUNT;axis++)pid_enabled[axis]=0U;
	      roll_control.integral = 0.0f;
	      pitch_control.integral = 0.0f;
	      yaw_control.integral = 0.0f;
	      static uint8_t clear_message[] =
	          "ACK CMD=EMERGENCY_STOP_CLEAR ACTIVE=0 PID=0,0,0\r\n";
	      TelemetryProtocol_Event(clear_message, sizeof(clear_message) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_CONTROL_ATTITUDE)
	    {
	      control_mode = CONTROL_MODE_ATTITUDE;
	      for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	      {
	        wheel_speed_control[axis].integral = 0.0f;
	        wheel_speed_control[axis].output_pwm = 0.0f;
	      }
	      Motors_StopAll();
	      static uint8_t mode_message[] =
	          "ACK CMD=CTRL_MODE VALUE=ATTITUDE FEEDBACK=IMU ENCODER=SUPERVISOR\r\n";
	      TelemetryProtocol_Event(mode_message, sizeof(mode_message) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_CONTROL_WHEEL_SPEED)
	    {
	      /* Command sign does not measure wheel direction during coasting or
	         reversal. Do not expose a signed closed loop with invented feedback. */
	      control_mode = CONTROL_MODE_ATTITUDE;
	      /* Keep the operator PID request across calibration, faults and recovery. */
	      roll_control.integral = 0.0f;
	      pitch_control.integral = 0.0f;
	      yaw_control.integral = 0.0f;
	      Motors_StopAll();
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "NACK CMD=CTRL_MODE REASON=SIGNED_RPM_UNOBSERVABLE_1CH "
	          "GAINS_READY=%u,%u,%u DIR=ASSUMED_1CH\r\n",
	          wheel_speed_control[MOTOR_AXIS_ROLL].gains_configured,
	          wheel_speed_control[MOTOR_AXIS_PITCH].gains_configured,
	          wheel_speed_control[MOTOR_AXIS_YAW].gains_configured);
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_SET_WHEEL_RPM)
	    {
	      for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	      {
	        wheel_speed_control[axis].setpoint_rpm = pending_wheel_rpm[axis];
	        wheel_speed_control[axis].integral = 0.0f;
	      }
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=SET_WHEEL_RPM SP_m=%ld,%ld,%ld\r\n",
	          (long)(pending_wheel_rpm[0] * 1000.0f),
	          (long)(pending_wheel_rpm[1] * 1000.0f),
	          (long)(pending_wheel_rpm[2] * 1000.0f));
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_SET_WHEEL_PID)
	    {
	      WheelSpeedControl_t *wheel_pid =
	          &wheel_speed_control[pending_wheel_axis];
	      wheel_pid->kp = pending_wheel_pid_gain[0];
	      wheel_pid->ki = pending_wheel_pid_gain[1];
	      wheel_pid->kd = pending_wheel_pid_gain[2];
	      wheel_pid->integral = 0.0f;
	      wheel_pid->gains_configured =
	          ((wheel_pid->kp > 0.0f)
	           || (wheel_pid->ki > 0.0f)
	           || (wheel_pid->kd > 0.0f)) ? 1U : 0U;
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=WHEEL_PID AXIS=%u KP_u6=%ld KI_u6=%ld KD_u6=%ld READY=%u\r\n",
	          pending_wheel_axis,
	          (long)(wheel_pid->kp * 1000000.0f),
	          (long)(wheel_pid->ki * 1000000.0f),
	          (long)(wheel_pid->kd * 1000000.0f),
	          wheel_pid->gains_configured);
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_SET_RPM_LIMIT)
	    {
	      memcpy(wheel_rpm_limit, pending_rpm_limit, sizeof(wheel_rpm_limit));
	      int length = snprintf(
	          message,
	          sizeof(message),
	          "ACK CMD=RPM_LIMIT VALUE_m=%ld,%ld,%ld\r\n",
	          (long)(wheel_rpm_limit[0] * 1000.0f),
	          (long)(wheel_rpm_limit[1] * 1000.0f),
	          (long)(wheel_rpm_limit[2] * 1000.0f));
	      TelemetryProtocol_Event((uint8_t *)message, length);
	    }
	    else if (usb_command == USB_COMMAND_ENCODER_CLEAR)
	    {
	      uint8_t can_clear=1U;
	      for(unsigned axis=0;axis<MOTOR_AXIS_COUNT;axis++) {
	        if(!WheelEncoder_CanClear((MotorAxis_t)axis))can_clear=0U;
	      }
	      if(!can_clear) {
	        const uint8_t error[]="NACK CMD=ENCODER_CLEAR REASON=DISARM_AND_VERIFY_25_PULSES_PER_FAULTED_AXIS\r\n";
	        TelemetryProtocol_Event(error,sizeof(error)-1U);
	      } else {
	      for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	      {
	        wheel_encoder[axis].fault_flags = 0U;
	        wheel_encoder[axis].recovery_pulses = 0U;
	      }
	      static uint8_t clear_encoder_message[] =
	          "ACK CMD=ENCODER_CLEAR FAULT=0,0,0\r\n";
	      TelemetryProtocol_Event(
	          clear_encoder_message, sizeof(clear_encoder_message) - 1U);
	      }
	    }
	    else if (usb_command == USB_COMMAND_CAL_ACCEL_CANCEL)
	    {
	      memset(&accel_calibration, 0, sizeof(accel_calibration));
	      Motors_StopAll();
	      static uint8_t cancel_message[] =
	          "CAL STATUS TYPE=ACCEL PHASE=CANCELLED\r\n";
	      TelemetryProtocol_Event(cancel_message, sizeof(cancel_message) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_ZERO_YAW)
	    {
	      AttitudeControl_HoldCurrentYaw();
	      Motors_StopAll();
	      static uint8_t zero_message[] =
	          "ACK CMD=ZERO_YAW SETPOINT=CURRENT\r\n";
	      TelemetryProtocol_Event(zero_message, sizeof(zero_message) - 1U);
	    }
	    else if ((mpu_status != HAL_OK) || (usb_command==USB_COMMAND_CAL_MAG && mag_status!=HAL_OK))
	    {
	      static uint8_t sensor_error[] =
	          "CAL ERROR TYPE=SYSTEM REASON=SENSOR_NOT_READY\r\n";
	      TelemetryProtocol_Event(sensor_error, sizeof(sensor_error) - 1U);
	    }
	    else if (usb_command == USB_COMMAND_CAL_ACCEL_START)
	    {
	      {const uint8_t ack[]="ACK CMD=CAL_ACCEL_START STATE=STARTED\r\n";TelemetryProtocol_Event(ack,sizeof(ack)-1U);}
	      AccelSixPoint_Start();
	    }
	    else if (usb_command == USB_COMMAND_CAL_ACCEL_CAPTURE)
	    {
	      {const uint8_t ack[]="ACK CMD=CAL_ACCEL_CAPTURE STATE=STARTED\r\n";TelemetryProtocol_Event(ack,sizeof(ack)-1U);}
	      int32_t capture_result = AccelSixPoint_Capture();
	      if (capture_result > 0)
	      {
	        static uint8_t still_message[] =
	            "CAL STATUS TYPE=ACCEL PHASE=COMPLETE KEEP_SYSTEM_STILL_FOR_GYRO\r\n";
	        TelemetryProtocol_Event(still_message, sizeof(still_message) - 1U);
	        App_Wait(1000);
	        fusion_status = Fusion_RestartAfterStationaryCalibration(
	            calibrated_gyro_bias,
	            initial_accel_average);
	      }
	    }
	    else if (usb_command == USB_COMMAND_CAL_MAG)
	    {
	      {const uint8_t ack[]="ACK CMD=CAL_MAG STATE=STARTED\r\n";TelemetryProtocol_Event(ack,sizeof(ack)-1U);}
	      memset(&accel_calibration, 0, sizeof(accel_calibration));
	      Motors_StopAll();
	      /* Keep the operator PID request across calibration, faults and recovery. */
	      /* Failed acquisition must not corrupt a working calibration. */
	      if(GY511_MagCalibrate()==HAL_OK) {
	        const uint8_t reference[]="CAL STATUS TYPE=MAG PHASE=REFERENCE KEEP_SYSTEM_STILL\r\n";
	        TelemetryProtocol_Event(reference,sizeof(reference)-1U);App_Wait(1000);
	        fusion_status=Fusion_CalibrateAndStart(calibrated_gyro_bias,initial_accel_average);
	      } else {
	        const uint8_t error[]="CAL ERROR TYPE=MAG REASON=INSUFFICIENT_COVERAGE_OLD_CAL_RETAINED\r\n";
	        TelemetryProtocol_Event(error,sizeof(error)-1U);
	      }
	    }
	    else if (usb_command == USB_COMMAND_CAL_GYRO)
	    {
	      {const uint8_t ack[]="ACK CMD=CAL_GYRO STATE=STARTED\r\n";TelemetryProtocol_Event(ack,sizeof(ack)-1U);}
	      memset(&accel_calibration, 0, sizeof(accel_calibration));
	      fusion_status = Fusion_RestartAfterStationaryCalibration(
	          calibrated_gyro_bias,
	          initial_accel_average);
	    }

	    now = HAL_GetTick();
	    /* STATUS/target bursts must not continually postpone the PID tick. */
	    if(now-command_started_ms>=CONTROL_PERIOD_MS) {
	      last_control_tick=now;last_mag_tick=now;last_encoder_control_tick=now;
	    }
	  }

	  if ((mpu_status != HAL_OK) || (fusion_status != HAL_OK))
	  {
	    Motors_StopAll();
	    int length = snprintf(
	        message,
	        sizeof(message),
	        "SENSOR ERROR MPU=%u WHO=0x%02X MAG=%u ADDR=0x%02X FUSION=%u I2CERR=0x%08lX SCL=%u SDA=%u BUSY=%u\r\n",
	        (unsigned int)mpu_status,
	        mpu_who_am_i,
	        (unsigned int)mag_status,
	        (unsigned int)(gy511_mag_address >> 1),
	        (unsigned int)fusion_status,
	        HAL_I2C_GetError(&hi2c1),
	        (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6),
	        (unsigned int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7),
	        (unsigned int)((I2C1->SR2 & I2C_SR2_BUSY) != 0U));

	    TelemetryProtocol_Event((uint8_t *)message, length);
	    App_Wait(250);

	    if ((int32_t)(HAL_GetTick() - next_scan_tick) >= 0)
	    {
	      I2C1_BusRecover();
	      I2C_ScanAndReport();
	      next_scan_tick = HAL_GetTick() + 5000U;
	    }

	    /* Retry after releasing a bus that may have been left busy. */
	    I2C1_BusRecover();
	    mpu_status = MPU6050_Init(&mpu_who_am_i);
	    mag_status = GY511_MagInit();
	    if(!calibration_gyro_valid)Calibration_Load();
	    bmp280_status = BMP280_Init();
	    ina219_status = INA219_Init();
	    aht30_status = AHT30_Init();
	    if (mpu_status == HAL_OK)
	    {
	      fusion_status = Fusion_CalibrateAndStart(
	          calibrated_gyro_bias,
	          initial_accel_average);
	      last_control_tick = HAL_GetTick();
	      last_mag_tick = last_control_tick;
	      last_encoder_control_tick = last_control_tick;
	      mag_read_failures = 0U;
	    }
	  }
	  else if (accel_calibration.active != 0U)
	  {
	    Motors_StopAll();
	  }
	  else if ((now - last_control_tick) >= CONTROL_PERIOD_MS)
	  {
	    control_actual_dt_us=(now-last_control_tick)*1000U;
	    if(now-last_control_tick>10U)control_late++;
	    float dt = (float)(now - last_control_tick) * 0.001f;
	    last_control_tick = now;
	    if(!ControlTiming_Accept(control_actual_dt_us/1000U,now)) {
	      Motors_StopAll();
	      /* Keep the operator PID request across calibration, faults and recovery. */
	      fusion_status=HAL_ERROR; /* missing motion interval: explicit reinitialization */
	      const uint8_t error[]="NACK CMD=CONTROL REASON=DEADLINE_MISSED_OUTPUT_BLOCKED_PID_REQUEST_RETAINED\r\n";
	      TelemetryProtocol_Event(error,sizeof(error)-1U);
	      continue;
	    }
	    uint8_t encoder_control_due = 0U;
	    float encoder_dt = 0.0f;
	    if ((now - last_encoder_control_tick) >= ENCODER_CONTROL_PERIOD_MS)
	    {
	      encoder_dt = (float)(now - last_encoder_control_tick) * 0.001f;
	      last_encoder_control_tick = now;
	      WheelEncoders_Update(now);
	      encoder_control_due = 1U;
	    }

	    if (MPU6050_ReadRaw(
	            &ax, &ay, &az,
	            &temperature,
	            &gx, &gy, &gz) == HAL_OK)
	    {
	      last_imu_raw[0]=ax;last_imu_raw[1]=ay;last_imu_raw[2]=az;
	      last_imu_raw[3]=gx;last_imu_raw[4]=gy;last_imu_raw[5]=gz;
	      last_imu_sample_ms=HAL_GetTick();
	      AttitudeEKF_Update(ax, ay, az, gx, gy, gz, dt);

	      if ((now - last_mag_tick) >= MAG_UPDATE_PERIOD_MS)
	      {
	        int16_t mx=0,my=0,mz=0;
	        last_mag_tick = now;
	        HAL_StatusTypeDef read_status = GY511_MagReadRaw(&mx, &my, &mz);
	        Magnetometer_ProcessSample(read_status,mx,my,mz);
	      }

	      float body_gyro_rad_s[3];
	      SensorToBodyGyroRad((float)gx, (float)gy, (float)gz, body_gyro_rad_s);
	      float roll_axis_rate_dps =
	          (body_gyro_rad_s[0] - attitude_ekf.gyro_bias_rad_s[0]) * RAD_TO_DEG;
	      float pitch_axis_rate_dps =
	          (body_gyro_rad_s[1] - attitude_ekf.gyro_bias_rad_s[1]) * RAD_TO_DEG;
	      float yaw_axis_rate_dps =
	          (YAW_AXIS_X * (body_gyro_rad_s[0] - attitude_ekf.gyro_bias_rad_s[0])
	         + YAW_AXIS_Y * (body_gyro_rad_s[1] - attitude_ekf.gyro_bias_rad_s[1])
	         + YAW_AXIS_Z * (body_gyro_rad_s[2] - attitude_ekf.gyro_bias_rad_s[2]))
	          * RAD_TO_DEG;

	      float target_deg[3]={roll_control.setpoint_deg,pitch_control.setpoint_deg,yaw_control.setpoint_deg};
	      /* Mag loss/rejection changes estimator health only. Keep requested PID
	         enable, target, quaternion, integral and IMU-based feedback intact. */
	      float body_error[3]={0};
	      int error_valid=ADCS_AttitudeError(attitude_ekf.q,target_deg,body_error);
	      float yaw_error=YAW_AXIS_X*body_error[0]+YAW_AXIS_Y*body_error[1]+YAW_AXIS_Z*body_error[2];
	      uint8_t attitude_mode_enabled = (control_mode == CONTROL_MODE_ATTITUDE) && !emergency_stop && error_valid && fusion_status==HAL_OK
	          && !control_timing_paused;
	      int32_t roll_command = AxisControl_Update(
	          &roll_control,
	          attitude_ekf.roll_deg,
	          body_error[0],
	          roll_axis_rate_dps,
	          ROLL_KP, ROLL_KI, ROLL_KD,
	          ROLL_INTEGRAL_LIMIT,
	          ROLL_MOTOR_CONTROL_SIGN,
	          dt,
	          attitude_mode_enabled && pid_enabled[MOTOR_AXIS_ROLL]);
	      int32_t pitch_command = AxisControl_Update(
	          &pitch_control,
	          attitude_ekf.pitch_deg,
	          body_error[1],
	          pitch_axis_rate_dps,
	          PITCH_KP, PITCH_KI, PITCH_KD,
	          PITCH_INTEGRAL_LIMIT,
	          PITCH_MOTOR_CONTROL_SIGN,
	          dt,
	          attitude_mode_enabled && pid_enabled[MOTOR_AXIS_PITCH]);
	      int32_t yaw_command = AxisControl_Update(
	          &yaw_control,
	          attitude_ekf.yaw_deg,
	          yaw_error,
	          yaw_axis_rate_dps,
	          YAW_KP, YAW_KI, YAW_KD,
	          YAW_INTEGRAL_LIMIT,
	          YAW_MOTOR_CONTROL_SIGN,
	          dt,
	          attitude_mode_enabled && pid_enabled[MOTOR_AXIS_YAW]);
	      attitude_ekf.yaw_rate_dps = yaw_control.rate_dps;

	      if ((control_mode == CONTROL_MODE_WHEEL_SPEED_TEST)
	          && (encoder_control_due != 0U))
	      {
	        for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	        {
	          if (pid_enabled[axis] != 0U)
	          {
	            (void)WheelSpeedControl_Update((MotorAxis_t)axis, encoder_dt);
	          }
	          else
	          {
	            wheel_speed_control[axis].integral = 0.0f;
	            wheel_speed_control[axis].output_pwm = 0.0f;
	          }
	        }
	      }

	      int32_t requested_command[MOTOR_AXIS_COUNT];
	      if (control_mode == CONTROL_MODE_ATTITUDE)
	      {
	        requested_command[MOTOR_AXIS_ROLL] = roll_command;
	        requested_command[MOTOR_AXIS_PITCH] = pitch_command;
	        requested_command[MOTOR_AXIS_YAW] = yaw_command;
	      }
	      else
	      {
	        requested_command[MOTOR_AXIS_ROLL] =
	            (int32_t)wheel_speed_control[MOTOR_AXIS_ROLL].output_pwm;
	        requested_command[MOTOR_AXIS_PITCH] =
	            (int32_t)wheel_speed_control[MOTOR_AXIS_PITCH].output_pwm;
	        requested_command[MOTOR_AXIS_YAW] =
	            (int32_t)wheel_speed_control[MOTOR_AXIS_YAW].output_pwm;
	      }

	      if (emergency_stop != 0U)
	      {
	        Motors_StopAll();
	      }
	      else
	      {
	        for (uint32_t axis = 0U; axis < MOTOR_AXIS_COUNT; axis++)
	        {
	          int32_t applied = Motor_ApplySupervisedCommand(
	              (MotorAxis_t)axis, requested_command[axis]);
	          if ((applied != requested_command[axis])
	              && (control_mode == CONTROL_MODE_ATTITUDE))
	          {
	            AxisControl_t *attitude_axis = (axis == MOTOR_AXIS_ROLL)
	                ? &roll_control
	                : ((axis == MOTOR_AXIS_PITCH)
	                    ? &pitch_control : &yaw_control);
	            attitude_axis->integral = 0.0f;
	            attitude_axis->saturated = 1U;
	          }
	        }
	      }

	    }
	    else
	    {
	      Motors_StopAll();
	      mpu_status = HAL_ERROR;
	    }
	  }

	  /* Optional environmental/current sensors are serviced after the
	     reaction-wheel control work with a short, bounded I2C timeout. */
	  if(!Magnetometer_RecoveryService(HAL_GetTick()))SlowSensors_Service(now);
	  App_CommunicationService(0);
	  App_Wait(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USART1 RX interrupt only latches stop and clears motor registers. No parser,
   printf, I2C, DMA restart or ACK formatting occurs in this interrupt hook. */
void ADCS_FaultStop(void)
{
  emergency_stop=1U;
  if(motor_initialized) {
    TIM3->CCR1=0;TIM3->CCR2=0;TIM3->CCR3=0;
    motor_command[0]=motor_command[1]=motor_command[2]=0;
    GPIOC->BSRR=(uint32_t)(GPIO_PIN_4|GPIO_PIN_5)<<16U;
    GPIOB->BSRR=(uint32_t)(GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13)<<16U;
  }
}
void TelemetryUART_EstopIRQ(void){ADCS_FaultStop();}
static void App_CommunicationService(uint8_t busy)
{
  uint8_t bytes[256];
  TelemetryUART_Service();
  if(TelemetryUART_TakeRxLoss())CommandParser_DiscardPartial();
  if(TelemetryUART_TakeEstop()) {
    emergency_stop=1U;Motors_StopAll();CommandParser_Clear();
      for(unsigned axis=0;axis<MOTOR_AXIS_COUNT;axis++)pid_enabled[axis]=0U;
    const uint8_t ack[]="ACK CMD=EMERGENCY_STOP ACTIVE=1\r\n";
    TelemetryProtocol_Event(ack,sizeof(ack)-1U);
  }
  for(unsigned pass=0;pass<4;pass++) {
    size_t n=TelemetryUART_Read(bytes,sizeof(bytes));if(!n)break;
    CommandStats before=CommandParser_Stats();
    CommandParser_Feed(bytes,n,HAL_GetTick());
    CommandStats after=CommandParser_Stats();
    if(after.malformed!=before.malformed) {
      const uint8_t error[]="NACK CMD=RX REASON=MALFORMED\r\n";
      TelemetryProtocol_Event(error,sizeof(error)-1U);
    }
    if(after.queue_dropped!=before.queue_dropped) {
      const uint8_t error[]="NACK CMD=RX REASON=QUEUE_FULL\r\n";
      TelemetryProtocol_Event(error,sizeof(error)-1U);
    }
  }
  if(busy) {
    ParsedCommand cmd;
    while(CommandParser_Pop(&cmd)) {
      if(cmd.type==USB_COMMAND_PID){
        if(cmd.enabled && emergency_stop){
          const uint8_t reject[]="NACK CMD=PID REASON=ESTOP_ACTIVE\r\n";
          TelemetryProtocol_Event(reject,sizeof(reject)-1U);
        }else{
          for(unsigned i=0;i<MOTOR_AXIS_COUNT;i++)if(cmd.axis==i || cmd.axis==MOTOR_AXIS_COUNT){
            pid_enabled[i]=cmd.enabled;if(!cmd.enabled)Motor_SetAxisCommand((MotorAxis_t)i,0);
          }
          char ack[128];int n=snprintf(ack,sizeof(ack),"ACK CMD=PID ENABLE=%u STATE=%u,%u,%u AUTHORITY=OPERATOR BUSY=1\r\n",cmd.enabled,pid_enabled[0],pid_enabled[1],pid_enabled[2]);
          TelemetryProtocol_Event((uint8_t *)ack,(size_t)n);
        }
        continue;
      }
      if(cmd.type==USB_COMMAND_ESTOP) {emergency_stop=1U;Motors_StopAll();for(unsigned i=0;i<MOTOR_AXIS_COUNT;i++)pid_enabled[i]=0U;}
      else if(cmd.type==USB_COMMAND_CAL_ACCEL_CANCEL)accel_calibration.active=0U;
      const char *reply=cmd.type==USB_COMMAND_STATUS?"ACK CMD=STATUS BUSY=1\r\n":
          (cmd.type==USB_COMMAND_ESTOP?"ACK CMD=EMERGENCY_STOP ACTIVE=1\r\n":
          (cmd.type==USB_COMMAND_CAL_ACCEL_CANCEL?"CAL STATUS TYPE=ACCEL PHASE=CANCELLED\r\n":
          "NACK CMD=BUSY REASON=CALIBRATION_OR_SENSOR_RECOVERY\r\n"));
      TelemetryProtocol_Event((const uint8_t *)reply,strlen(reply));
    }
  }
  static uint32_t snapshot_tick;
  uint32_t now=HAL_GetTick();
  if(now-snapshot_tick>=10U) {
    snapshot_tick=now;App_BuildSnapshot(&telemetry_data);
    TelemetryProtocol_Service(&telemetry_data);
  }
  TelemetryUART_Service();
}
static void App_Wait(uint32_t milliseconds)
{
  uint32_t start=HAL_GetTick();uint8_t previous=app_busy;
  if(milliseconds>1U)app_busy=1U;
  do {App_CommunicationService(app_busy);HAL_Delay(1);}while(HAL_GetTick()-start<milliseconds);
  app_busy=previous;
}
static void App_BuildSnapshot(TelemetryData *s)
{
  memset(s,0,sizeof(*s));
  s->uptime_ms=HAL_GetTick();s->who=mpu_who_am_i;s->fusion=(uint8_t)fusion_status;
  s->imu_age_ms=last_imu_sample_ms?s->uptime_ms-last_imu_sample_ms:UINT32_MAX;
  s->attitude_valid=(mpu_status==HAL_OK && fusion_status==HAL_OK
      && s->imu_age_ms<=20U && !app_busy && !accel_calibration.active)?1U:0U;
  float accel[3],gyro[3];
  SensorToBodyAccel(last_imu_raw[0],last_imu_raw[1],last_imu_raw[2],accel);
  SensorToBodyGyroRad(last_imu_raw[3],last_imu_raw[4],last_imu_raw[5],gyro);
  AxisControl_t *axis[3]={&roll_control,&pitch_control,&yaw_control};
  const float measured[3]={attitude_ekf.roll_deg,attitude_ekf.pitch_deg,attitude_ekf.yaw_deg};
  for(unsigned i=0;i<3;i++) {
    s->angle[i]=(int32_t)(measured[i]*1000.0f);s->target[i]=(int32_t)(axis[i]->setpoint_deg*1000);
    s->error[i]=(int32_t)(WrapAngle180(axis[i]->setpoint_deg-measured[i])*1000);
    s->body_error[i]=(int32_t)(axis[i]->error_deg*1000);
    s->gyro_bias[i]=(int32_t)(attitude_ekf.gyro_bias_rad_s[i]*RAD_TO_DEG*1000);
    s->gyro[i]=(int32_t)((gyro[i]-attitude_ekf.gyro_bias_rad_s[i])*RAD_TO_DEG*1000);
    s->accel[i]=(int32_t)(accel[i]*1000/ACCEL_LSB_PER_G);s->mag[i]=(int32_t)magnetometer.body[i];
    s->pid_output[i]=(int32_t)axis[i]->output_pwm;s->requested_pwm[i]=requested_motor_command[i];s->pwm[i]=motor_command[i];
    s->rpm_milli[i]=(int32_t)(wheel_encoder[i].rpm*1000);s->count[i]=wheel_encoder[i].count;
    s->raw_rpm[i]=(int32_t)ClampFloat(wheel_encoder[i].raw_rpm*1000,0,2147483520.0f);
    s->encoder_age[i]=wheel_encoder[i].pulse_seen?s->uptime_ms-wheel_encoder[i].last_edge_tick_ms:UINT32_MAX;
    s->encoder_angle[i]=(int32_t)(wheel_encoder[i].angle_deg*1000);s->encoder_rejected[i]=wheel_encoder[i].rejected_samples;
    s->p_term[i]=(int32_t)(axis[i]->p_term*1000);s->i_term[i]=(int32_t)(axis[i]->i_term*1000);s->d_term[i]=(int32_t)(axis[i]->d_term*1000);
    s->rpm_signed[i]=(int32_t)(wheel_encoder[i].signed_rpm_estimate*1000);s->rpm_target[i]=(int32_t)(wheel_speed_control[i].setpoint_rpm*1000);
    s->rpm_error[i]=s->rpm_target[i]-s->rpm_signed[i];s->rpm_limit[i]=(int32_t)(wheel_rpm_limit[i]*1000);
    s->encoder_valid[i]=wheel_encoder[i].speed_valid;s->encoder_fault[i]=wheel_encoder[i].fault_flags;s->encoder_limit[i]=wheel_encoder[i].overspeed_limited;
    s->encoder_recovery[i]=wheel_encoder[i].recovery_pulses;
    s->wheel_ready[i]=wheel_speed_control[i].gains_configured;s->wheel_kp[i]=(int32_t)(wheel_speed_control[i].kp*1000000);
    s->wheel_ki[i]=(int32_t)(wheel_speed_control[i].ki*1000000);s->wheel_kd[i]=(int32_t)(wheel_speed_control[i].kd*1000000);s->pid[i]=pid_enabled[i];
    /* Requested state and output permission are intentionally separate. */
    s->pid_block[i]=(emergency_stop?1U:0U)|(mpu_status!=HAL_OK || s->imu_age_ms>20U?2U:0U)
        |(fusion_status!=HAL_OK?4U:0U)|(app_busy || accel_calibration.active?8U:0U)
        |(wheel_encoder[i].fault_flags?16U:0U)|(!wheel_encoder[i].speed_valid?32U:0U)
        |(wheel_encoder[i].overspeed_limited?64U:0U)
        |(control_timing_paused && (int32_t)(s->uptime_ms-control_pause_until_ms)<0?128U:0U);
    s->pid_active[i]=pid_enabled[i] && !s->pid_block[i];
  }
  /* Operator confirmed: Roll=PA8, Pitch=PC6, Yaw=PA0, all 25 pulses/rev.
   * Presence reporting does not bypass motor/encoder fault supervision. */
  s->encoder_installed[0]=1U;s->encoder_installed[1]=1U;s->encoder_installed[2]=1U;
  s->encoder_pin[0]=(GPIOA->IDR & GPIO_PIN_8)?1U:0U;
  s->encoder_pin[1]=(GPIOC->IDR & GPIO_PIN_6)?1U:0U;
  s->encoder_pin[2]=(GPIOA->IDR & GPIO_PIN_0)?1U:0U;
  s->mag_ratio_milli=(int32_t)(mag_field_ratio*1000);
  s->mag_innovation_mdeg=(int32_t)(mag_innovation_deg*1000);
  for(unsigned i=0;i<4;i++)s->quaternion[i]=(int32_t)(attitude_ekf.q[i]*1000000);
  s->yaw_rate=(int32_t)(yaw_control.rate_dps*1000);s->mag_rejected=attitude_ekf.mag_rejected;s->mag_accepted=attitude_ekf.mag_accepted;
  s->accel_accepted=attitude_ekf.accel_accepted;s->accel_rejected=attitude_ekf.accel_rejected;
  s->mag_age_ms=last_heading_update_ms?s->uptime_ms-last_heading_update_ms:0xFFFFFFFFU;s->mag_quality=magnetic_heading_quality;
  s->mag_sample_age_ms=last_mag_sample_ms?s->uptime_ms-last_mag_sample_ms:UINT32_MAX;
  /* Diagnostic state, never an actuator-disarm request. */
  s->mag_state=mag_status!=HAL_OK?4U:(!magnetometer.reference_valid?0U:
      (s->mag_sample_age_ms>200U?3U:(mag_reject_reason || s->mag_age_ms>500U?2U:1U)));
  s->mag_reason=mag_reject_reason;s->mag_io_errors=mag_io_errors;
  s->calibration_source=calibration_from_flash;s->calibration_measured=calibration_measured_flags;
  s->calibration_save_result=(uint8_t)calibration_save_result;
  s->bmp_centi_c=bmp280.temperature_centi_c;s->pressure_pa=bmp280.pressure_pa;
  static uint32_t altitude_pressure;
  if(bmp280.valid && bmp280.pressure_pa>10000U && bmp280.pressure_pa!=altitude_pressure) {
    altitude_pressure=bmp280.pressure_pa;
    if(!pressure_reference_pa)pressure_reference_pa=bmp280.pressure_pa;
    relative_altitude_mm=(int32_t)(44330000.0f*(1.0f-powf((float)bmp280.pressure_pa/(float)pressure_reference_pa,0.19029495f)));
  }
  s->altitude_mm=relative_altitude_mm;s->voltage_mv=ina219.bus_mv;s->current_ma10=ina219.current_ma_x10;s->power_mw=ina219.power_mw;
  s->battery_percent=ina219.valid?(int32_t)ClampFloat((float)(ina219.bus_mv-9600)*100/3000,0,100):-1;
  s->aht_milli_c=aht30.temperature_milli_c;s->humidity_milli=aht30.humidity_milli_percent;
  s->sv=(bmp280.valid?1U:0U)|(ina219.valid?2U:0U)|(aht30.valid?4U:0U);
  const uint32_t max_env_age[3]={3U*BMP280_UPDATE_PERIOD_MS,3U*INA219_UPDATE_PERIOD_MS,
      3U*AHT30_UPDATE_PERIOD_MS+AHT30_MEASUREMENT_TIME_MS};
  for(unsigned i=0;i<3;i++) {
    s->env_age_ms[i]=last_env_sample_ms[i]?s->uptime_ms-last_env_sample_ms[i]:UINT32_MAX;
    if(s->env_age_ms[i]>max_env_age[i])s->sv &= ~(1U<<i);
  }
  s->sensor_error=(mpu_status!=HAL_OK?1U:0U)|(mag_status!=HAL_OK?2U:0U)|(fusion_status!=HAL_OK?4U:0U)
      |(bmp280_status!=HAL_OK?8U:0U)|(ina219_status!=HAL_OK?16U:0U)|(aht30_status!=HAL_OK?32U:0U);
  s->dt_us=attitude_ekf.last_dt_us;s->control_dt_us=control_actual_dt_us;s->control_late=control_late;
  s->gyro_clip_count=attitude_ekf.gyro_clipped_samples;s->accel_clip_count=attitude_ekf.accel_clipped_samples;
  s->accel_noise_u6=(uint32_t)(attitude_ekf.accel_observation_variance*1000000.0f);
  s->stationary=attitude_ekf.stationary;s->estop=emergency_stop;s->charging=system_mode==SYSTEM_MODE_CHARGING;s->speed_mode=control_mode==CONTROL_MODE_WHEEL_SPEED_TEST;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  ADCS_FaultStop();
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
