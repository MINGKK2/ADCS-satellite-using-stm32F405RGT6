// Hamilton quaternion [x,y,z,w], active body-to-world rotation, intrinsic ZYX.
// Inputs are measured Euler angles in degrees, never target/setpoint values.
export function attitudeQuaternion(roll,pitch,yaw) {
  if(![roll,pitch,yaw].every(Number.isFinite))return null;
  const h=Math.PI/360,cr=Math.cos(roll*h),sr=Math.sin(roll*h);
  const cp=Math.cos(pitch*h),sp=Math.sin(pitch*h),cy=Math.cos(yaw*h),sy=Math.sin(yaw*h);
  return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy];
}
