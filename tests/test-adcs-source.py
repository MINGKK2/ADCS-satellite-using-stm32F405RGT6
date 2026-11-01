"""Compile selected actual firmware functions with minimal host peripherals.
No reimplementation of PID, pulse counting or ESKF updates in the tests.
"""
from pathlib import Path
import re, subprocess,sys,os,shutil
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'firmware'
here=Path(__file__).parent
source=(root/'Core/Src/main.c').read_text(encoding='utf-8')
def function(name):
    match=re.search(r'^static [^\n]*\b'+name+r'\s*\(',source,re.M)
    assert match,name
    start=source.index('{',match.start()); depth=1; end=start+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[match.start():end]
def structure(name):
    return next(m.group() for m in re.finditer(r'typedef struct\s*\{[^}]+\}\s*\w+;',source) if m.group().endswith('} '+name+';'))
prefix='''#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include "heading_update.h"
#include "calibration_store.h"
typedef enum {MOTOR_AXIS_ROLL,MOTOR_AXIS_PITCH,MOTOR_AXIS_YAW,MOTOR_AXIS_COUNT} MotorAxis_t;
typedef struct {uint32_t CNT;} TIM_TypeDef;
typedef enum {HAL_OK,HAL_ERROR,HAL_BUSY} HAL_StatusTypeDef;
static uint32_t tick;
static uint32_t HAL_GetTick(void){return tick;}
static void App_Wait(uint32_t ms){tick+=ms;}
static int32_t motor_command[3],requested_motor_command[3];
static float wheel_rpm_limit[3]={4800,4800,4800};
static void Motor_SetAxisCommand(MotorAxis_t i,int32_t v){motor_command[i]=v;}
static void Motors_StopAll(void){memset(motor_command,0,sizeof(motor_command));}
static int TelemetryProtocol_Event(const uint8_t *p,size_t n){(void)p;(void)n;return 0;}
static void Calibration_Save(uint32_t flags){(void)flags;}
static HAL_StatusTypeDef GY511_MagReadRaw(int16_t*x,int16_t*y,int16_t*z){*x=3;*y=4;*z=5;return HAL_OK;}
'''
defines='\n'.join(re.findall(r'^#define [^\n]+',source,re.M))
globals='''
static AttitudeEkf_t attitude_ekf;
static WheelEncoder_t wheel_encoder[3];
static Magnetometer_t magnetometer;
static uint8_t emergency_stop,magnetic_heading_quality;
static uint8_t pid_enabled[3];
static HAL_StatusTypeDef fusion_status;
static HAL_StatusTypeDef mag_status;
static uint32_t mag_io_errors,mag_read_failures,last_mag_sample_ms,mag_reference_epoch;
static uint8_t mag_reject_reason;
static uint32_t last_heading_update_ms,clean_heading_samples;
static float mag_field_ratio,mag_innovation_deg;
static float accel_offset_raw[3]={ACCEL_OFFSET_X_RAW_DEFAULT,ACCEL_OFFSET_Y_RAW_DEFAULT,ACCEL_OFFSET_Z_RAW_DEFAULT};
static float accel_scale[3]={ACCEL_SCALE_X_DEFAULT,ACCEL_SCALE_Y_DEFAULT,ACCEL_SCALE_Z_DEFAULT};
'''
names=['ClampFloat','WrapAngle180','SensorToBodyAccel','SensorToBodyGyroRad',
       'QuaternionNormalize','QuaternionToEuler','Matrix3Inverse','AttitudeEKF_Init',
       'AttitudeEKF_PropagateCovariance','AttitudeEKF_AccelerometerUpdate',
       'RotateWorldToBody','RotateBodyToWorld','GY511_MagToBody','AttitudeEKF_ObserveInitialMagReference',
       'AttitudeEKF_MagnetometerUpdate','AttitudeEKF_RefreshEuler','Magnetometer_ProcessSample','AttitudeEKF_Update',
       'AxisControl_Update','WheelEncoder_UpdateOne','Motor_ApplySupervisedCommand','GY511_MagCalibrate']
program=prefix+defines+'\n'+''.join(structure(n)+'\n' for n in ['AttitudeEkf_t','AxisControl_t','WheelEncoder_t','Magnetometer_t'])+globals+'\n'.join(function(n) for n in names)
program+='\n'+(here/'test-adcs-cases.c').read_text()
generated=here/'adcs-source-generated.c';generated.write_text(program)
compiler=os.environ.get('ADCS_HOST_CC') or shutil.which('gcc') or r'C:\msys64\ucrt64\bin\gcc.exe'
exe=here/'test-adcs-source.exe'
subprocess.run([compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'Core/Inc'),str(generated),'-o',str(exe),'-lm'],check=True)
subprocess.run([str(exe)],check=True)
