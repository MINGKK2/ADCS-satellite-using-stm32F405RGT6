#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H
#include <stdint.h>
#define CALIBRATION_ACCEL 1U
#define CALIBRATION_MAG 2U
#define CALIBRATION_GYRO 4U
/* Bump the layout tag when sensor axis maps or units change. */
#define CALIBRATION_LAYOUT_TAG 0xF4053101UL
typedef struct {
  uint32_t layout_tag,flags,measured_flags,mpu_who,mag_type;
  float accel_offset[3],accel_scale[3],mag_offset[3],mag_scale[3],gyro_bias[3];
} CalibrationData;
typedef enum {CAL_STORE_OK=0,CAL_STORE_EMPTY=1,CAL_STORE_INVALID=2,
              CAL_STORE_FULL=3,CAL_STORE_IO_ERROR=4} CalibrationStoreResult;
int CalibrationStore_Valid(const CalibrationData *data);
CalibrationStoreResult CalibrationStore_Load(CalibrationData *data);
/* Append only, never erase: a failed save cannot destroy the previous record.
 * Call with motors stopped. Same coefficients do not consume another slot. */
CalibrationStoreResult CalibrationStore_Save(const CalibrationData *data);
#endif
