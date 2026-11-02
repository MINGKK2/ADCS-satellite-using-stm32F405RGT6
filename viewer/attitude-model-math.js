// Hamilton quaternion [x,y,z,w], active body-to-world rotation, intrinsic ZYX.
// Inputs are measured Euler angles in degrees, never target/setpoint values.
export function attitudeQuaternion(roll,pitch,yaw) {
  if(![roll,pitch,yaw].every(Number.isFinite))return null;
  const h=Math.PI/360,cr=Math.cos(roll*h),sr=Math.sin(roll*h);
  const cp=Math.cos(pitch*h),sp=Math.sin(pitch*h),cy=Math.cos(yaw*h),sy=Math.sin(yaw*h);
  return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy];
}

// Fast telemetry carries Hamilton[w,x,y,z]. Preserve its full3D rotation,
// including near the Euler pitch singularity; do not use a slow cached q.
export function poseQuaternion(state){
  if(state.quaternion===undefined)return attitudeQuaternion(state.roll,state.pitch,state.yaw);
  const q=state.quaternion;
  if(!Array.isArray(q)||q.length!==4||!q.every(Number.isFinite))return null;
  const norm=Math.hypot(...q),offset=state.yawOffset??0;
  if(norm<.5||norm>1.5||!Number.isFinite(offset))return null;
  const [w,x,y,z]=q.map(v=>v/norm),h=-offset*Math.PI/360,c=Math.cos(h),s=Math.sin(h);
  return [c*x-s*y,c*y+s*x,c*z+s*w,c*w-s*z];
}
