#ifndef ADCS_AXIS_FRAME_H
#define ADCS_AXIS_FRAME_H

/* Verified manual body rotations: physical Roll follows legacy +Y.
 * Rotate the body basis -90 degrees about Z: new=(oldY,-oldX,oldZ).
 * det(R)=+1; Yaw and vector cross products retain their handedness.
 * Both helpers permit in-place use.
 */
static inline void ADCS_LegacyToBody(const float old[3], float body[3])
{
  const float x=old[0], y=old[1], z=old[2];
  body[0]=y; body[1]=-x; body[2]=z;
}

static inline void ADCS_BodyToLegacy(const float body[3], float old[3])
{
  const float x=body[0], y=body[1], z=body[2];
  old[0]=-y; old[1]=x; old[2]=z;
}

/* Flash stores gyro bias in the historical body basis. Keep this encoding
 * stable so existing v1/v2-diagonal journals and firmware rollback remain
 * compatible. Accel/mag coefficients are raw-sensor coefficients already.
 */
static inline void ADCS_GyroBiasLoad(const float stored[3], float body[3])
{
  ADCS_LegacyToBody(stored,body);
}
static inline void ADCS_GyroBiasSave(const float body[3], float stored[3])
{
  ADCS_BodyToLegacy(body,stored);
}

#endif
