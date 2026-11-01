const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/attitude-viewer.html','utf8');
const elements=new Map([...html.matchAll(/id="([^"]+)"/g)].map(([,id])=>[id,{
  textContent:'',disabled:true,value:0,style:{},className:'',checked:id==='pidYaw',
  classList:{toggle(){},remove(){},add(){}},addEventListener(type,fn){this[type]=fn},
  querySelector(){return {textContent:''}}
}]));
let clock=100;
const ctx=vm.createContext({document:{getElementById(id){assert(elements.has(id),id);return elements.get(id)}},
  navigator:{},performance:{now:()=>clock},setInterval(){},setTimeout(){},console,TextEncoder});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],ctx);
vm.runInContext('setMissionControls(true)',ctx);
assert(!elements.get('sendTarget').disabled);
assert(!html.match(/autotune|autoTune|AT_MASK|RW_READY/i));
const frame='SEQ=1 R=0 P=0 Y=90000 TARGET_mdeg=0,0,0 BERR_mdeg=0,0,-90000 PIDOUT3=0,0,-255 REQPWM3=0,0,-255 PWM3=0,0,-255 ERPM3_m=0,0,120000 CTRL=ATTITUDE PID=0,0,1 ATK_u3=28000,1200,4000 YAW_MAG_AGE_ms=30 MAG_QUALITY=2 HOLD=FIXED_PID ATT_VALID=1';
vm.runInContext('render('+JSON.stringify(frame)+')',ctx);
assert.equal(elements.get('yaw').textContent,'90.00°');
assert.equal(vm.runInContext('cubesatAttitude.yaw',ctx),90);
assert.equal(vm.runInContext('cubesatAttitude.valid',ctx),true);
assert(elements.get('targetYaw').textContent.startsWith('0.00'));
assert(elements.get('dbgYawOutput').textContent.includes('-255 / -255 / -255'));
assert.equal(elements.get('pwm').textContent,'0 / 0 / -255');
assert.equal(elements.get('dbgYawRpm').textContent,'120.0');
assert(elements.get('yawReference').textContent.includes('chưa xác minh'));
vm.runInContext('render('+JSON.stringify(frame.replace('SEQ=1','SEQ=2').replace('YAW_MAG_AGE_ms=30','YAW_MAG_AGE_ms=5000'))+')',ctx);
assert.equal(elements.get('yaw').textContent,'90.00°');
assert(elements.get('yawReference').textContent.includes('có thể trôi'));
assert(elements.get('pidYaw').checked&&!elements.get('pidRoll').checked&&!elements.get('pidPitch').checked);
console.log('PASS: fixed PID UI, measured yaw separate from target, actual PWM/RPM and magnetic health; no Auto tune');
vm.runInContext("telemetryCache={};totalPackets=0;buffer='';discardRxLine=false;lastSequence=null;lastBoardTime=null;",ctx);
const fast='SEQ=1 T_ms=1000 R=1000 P=2000 Y=90000 TARGET_mdeg=0,0,45000 YR=1000 PWM3=0,0,50 REQPWM3=0,0,51 PIDOUT3=0,0,51 ERPM3_m=0,0,120000 PID=0,0,1 ATT_VALID=1\r\n';
for(let i=0;i<fast.length;i+=17)vm.runInContext('feedSerialText('+JSON.stringify(fast.slice(i,i+17))+')',ctx);
vm.runInContext('feedSerialText("ACK CMD=STATUS BUSY=0\\r\\nSLOW=1 T_ms=1000 WHO=0x70 BMP_TcC=2512 PPa=100123 INA_VmV=12000 SV=0x07 CAMERA=NOT_CONFIGURED\\r\\nDIAG=1 T_ms=1000 EC3=1,2,3 UART_ERR=0 TX_DROP=0 RX_DROP=0 CMD_BAD=0\\r\\n")',ctx);
assert.equal(elements.get('yaw').textContent,'90.00°');
assert.equal(elements.get('bmpTemp').textContent,'25.12 °C');
assert.equal(elements.get('encYawCount').textContent,3);
assert.equal(vm.runInContext('totalPackets',ctx),1); /* auxiliary frames do not fake fast-frame rate */
vm.runInContext('feedSerialText('+JSON.stringify('X'.repeat(5000))+')',ctx);
vm.runInContext('feedSerialText('+JSON.stringify('\n'+fast.replace('SEQ=1','SEQ=2'))+')',ctx);
assert.equal(vm.runInContext('totalPackets',ctx),2);
console.log('PASS: ESP-NOW-sized fragments, interleaved ACK/slow/diagnostic data, sensor cache and overlong-line recovery');
/* Fast telemetry can remain alive while slow/diagnostic streams are dead. */
clock=4000;
vm.runInContext('feedSerialText('+JSON.stringify(fast.replace('SEQ=1','SEQ=3').replace('T_ms=1000','T_ms=4000'))+')',ctx);
assert(elements.get('bmpState').textContent.includes('cũ'));
assert.equal(elements.get('dbgYawSafety').textContent,'CHẨN ĐOÁN CŨ');
assert.equal(elements.get('yaw').textContent,'90.00°');
clock=5001;vm.runInContext('updateFreshness()',ctx);
assert(elements.get('yaw').textContent.includes('cũ'));
assert.equal(elements.get('scene').style.opacity,'.35');
/* Board restart invalidates cached sensor state; repeated and reordered frames ignored. */
clock=5100;
vm.runInContext('feedSerialText('+JSON.stringify(fast.replace('T_ms=1000','T_ms=100'))+')',ctx);
assert.equal(vm.runInContext('telemetryCache.BMP_TcC',ctx),undefined);
let packets=vm.runInContext('totalPackets',ctx);
vm.runInContext('feedSerialText('+JSON.stringify(fast.replace('T_ms=1000','T_ms=100'))+')',ctx);
assert.equal(vm.runInContext('totalPackets',ctx),packets);
assert.equal(vm.runInContext('duplicates',ctx),1);
vm.runInContext('feedSerialText('+JSON.stringify(fast.replace('SEQ=1','SEQ=0').replace('T_ms=1000','T_ms=100'))+')',ctx);
assert.equal(vm.runInContext('reordered',ctx),1);
assert.equal(vm.runInContext('cubesatAttitude.roll',ctx),1);
assert.equal(vm.runInContext('cubesatAttitude.pitch',ctx),2);
assert.equal(vm.runInContext('cubesatAttitude.yaw',ctx),90);
console.log('PASS: independent stream ages, stale attitude, MCU restart cache invalidation, duplicate/reorder rejection, measured pose to 3D model');
/* Optional compass faults must not turn valid IMU data into an invalid EKF. */
clock=5200;
vm.runInContext('feedSerialText('+JSON.stringify(fast.replace('SEQ=1','SEQ=2').replace('T_ms=1000','T_ms=200'))+')',ctx);
vm.runInContext('feedSerialText("SLOW=1 T_ms=200 SENSOR_ERR=0x02 ATT_MODE=GYRO_ACCEL MAG_STATE=I2C_ERROR MAG_REASON=I2C CAL_SOURCE=FLASH CAL_MEASURED=7 CAL_SAVE=0\\r\\n")',ctx);
assert.equal(elements.get('yaw').textContent,'90.00°');
assert(elements.get('pidYaw').checked);
assert(elements.get('magState').textContent.includes('đang thử lại'));
assert(elements.get('sensorHealth').textContent.includes('Cảnh báo'));
assert(elements.get('yawReference').textContent.includes('PID không tự tắt'));
assert(elements.get('calStoreState').textContent.includes('Đã lưu Flash'));
vm.runInContext('feedSerialText("SLOW=1 T_ms=300 SENSOR_ERR=0x00 ATT_MODE=GYRO_ACCEL MAG_STATE=REJECTED MAG_REASON=FIELD_NORM CAL_SOURCE=FLASH CAL_MEASURED=7 CAL_SAVE=3\\r\\n")',ctx);
assert(elements.get('magState').textContent.includes('độ lớn từ trường'));
assert(elements.get('calStoreState').textContent.includes('chưa lưu'));
vm.runInContext('feedSerialText("SLOW=1 T_ms=400 SENSOR_ERR=0x00 ATT_MODE=9DOF MAG_STATE=TRACKING MAG_REASON=NONE CAL_SOURCE=FLASH CAL_MEASURED=7 CAL_SAVE=0\\r\\n")',ctx);
assert(elements.get('magState').textContent.includes('Đang hiệu chỉnh'));
assert.equal(elements.get('yaw').textContent,'90.00°');
console.log('PASS: optional compass warning, valid IMU/PID retained, rejection reason, persistent calibration status');
vm.runInContext('feedSerialText("SEQ=3 T_ms=500 R=0 P=0 Y=90000 PID=0,0,1 PID_ACTIVE=0,0,0 PID_BLOCK3=0,0,16 ATT_VALID=1\\r\\n")',ctx);
assert(elements.get('pidYaw').checked);
assert(elements.get('dbgYawSafety').textContent.includes('PID ON'));
assert(elements.get('dbgYawSafety').textContent.includes('fault encoder'));
vm.runInContext('feedSerialText("SEQ=4 T_ms=550 R=0 P=0 Y=90000 PID=0,0,1 PID_ACTIVE=0,0,1 PID_BLOCK3=0,0,0 ATT_VALID=1\\r\\n")',ctx);
assert.equal(elements.get('dbgYawSafety').textContent,'PID ON · đang điều khiển');
assert.equal(elements.get('yaw').textContent,'90.00°');
console.log('PASS operator PID ON remains checked during blocked output and resumes without a new ON command');
