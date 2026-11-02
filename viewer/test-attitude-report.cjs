const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const C=require('./attitude-report-core.js');
const meta='SLOW=1 FW=AXIS_FRAME_RP_PITCH_KP_V4 MAG_STATE=TRACKING';
function frame(i,t,y,target=90,extra=''){
  return `SEQ=${i} T_ms=${Math.round(t*1000)} R=0 P=0 Y=${Math.round(y*1000)} TARGET_mdeg=0,0,${target*1000} ATT_VALID=1 IMU_AGE_ms=4 PID=0,0,1 PID_ACTIVE=0,0,1 PID_BLOCK3=0,0,0 PWM3=0,0,100 REQPWM3=0,0,100 PIDOUT3=0,0,100 ERPM3_m=0,0,120000 G_mdps=0,0,0 BERR_mdeg=0,0,0 ESTOP=0 ${extra}`;
}
function recording(){const r=new C.Recorder();r.ingest(meta);r.ingest('DIAG=1 ENC_INSTALLED3=0,1,1 ATK_u3=11667,2400,8000');r.start('Synthetic step test');return r;}
const r=recording();
for(let i=0;i<=120;i++){
  const t=i/20,target=t<2?0:90;
  const y=t<2?0:t<3?90*(t-2):t<3.5?90+10*(t-3):t<4?95-10*(t-3.5):90;
  r.ingest(frame(i+1,t,y,target),100+t*1000,'2026-11-01T00:00:00.000Z');
}
let s=C.analyze(r.rows,2);
assert(s.reached&&s.step_observed);assert(Math.abs(s.settling_s-1.8)<1e-8);
assert.equal(s.overshoot_deg,5);assert.equal(s.final_mean_error_deg,0);assert(s.rmse_deg>10);
assert.equal(r.rows[0].angle[2],0);assert.equal(r.rows[40].target[2],90);
assert.equal(r.rows[40].angle[2],0); // target must never replace the measurement
assert.equal(r.rows[0].rpm[2],120);assert.equal(r.rows[0].gains[0],11.667);
const csv=C.toCsv(r),back=C.parseCsv(csv);
assert.equal(back.length,r.rows.length);assert.equal(back[80].angle[2],90);
assert.equal(C.analyze(back,2).overshoot_deg,5);assert(Math.abs(C.analyze(back,2).settling_s-1.8)<1e-8);
fs.mkdirSync(path.join(__dirname,'report-fixtures'),{recursive:true});
fs.writeFileSync(path.join(__dirname,'report-fixtures','synthetic-yaw-step.csv'),csv);
r.ingest(frame(122,6.05,95),6150);assert.equal(C.analyze(r.rows,2).reached,false);
r.ingest(frame(125,6.1,90),6200);assert(r.rows.at(-1).gapBefore);assert.equal(C.analyze(r.rows,2).reached,false);
r.ingest(frame(126,6.15,90).replace('ATT_VALID=1','ATT_VALID=0'),6250);assert(!r.rows.at(-1).valid);assert.equal(C.analyze(r.rows,2).settling_s,null);
r.ingest(frame(127,6.2,90).replace(' TARGET_mdeg=0,0,90000',''),6300);assert(!r.rows.at(-1).valid&&r.rows.at(-1).target.every(v=>v===null));
let n=r.rows.length;r.ingest(frame(127,6.2,90),6300);assert.equal(r.rows.length,n);
r.ingest(frame(126,6.19,90),6310);assert(!r.recording);assert.throws(()=>r.start('after reset'),/phiên mới/); // older board timestamp ends the epoch
r.clear();r.start('new epoch');r.ingest(frame(1,.1,0,0),10000);assert.equal(r.rows[0].t,0);
const w=recording();w.ingest(frame(4294967295,4294967.25,179,-179),100);w.ingest(frame(0,.004,-179,-179),150);
assert(w.recording&&w.rows.length===2);assert(Math.abs(w.rows[1].t-.05)<1e-8);assert.equal(w.rows[0].error[2],2);
const q=recording();q.ingest(frame(1,1,90),100);q.stop();q.ingest(frame(2,1.05,90),150);q.start('resume');q.ingest(frame(3,1.1,90),200);assert(q.rows[1].gapBefore);
const e=recording();e.ingest(frame(1,1,90),100);e.rows[0].fw='=HYPERLINK("x"),test';const escaped=C.toCsv(e);assert(escaped.includes("'=HYPERLINK"));assert.equal(C.parseCsv(escaped)[0].fw,"'=HYPERLINK(\"x\"),test");
assert.throws(()=>C.parseCsv('x,y\n1,2'),/định dạng/);
assert.throws(()=>C.parseCsv(csv.replace('0.05,0.05,','0,0.05,')),/Thời gian/);
const many=Array.from({length:10000},(_,i)=>({...back[0],t:i/20,valid:true,gapBefore:false,target:[0,0,90],angle:[0,0,i===5432?105:90]}));
many[0].angle[2]=0;
const picks=C.indices(many,2,'angle');assert(picks.includes(5432));assert(picks.length<2500);
assert.equal(C.analyze(many,2).overshoot_deg,15);
const opposite=back.slice(0,30).map((r,i)=>({...r,t:i/20,angle:[0,0,i%2?179:-179],target:[0,0,0]}));
const oppositeStats=C.analyze(opposite,2);assert(Math.abs(oppositeStats.final_mean_error_deg)>179);assert(!oppositeStats.reached);
const noStep=back.slice(90);assert(C.analyze(noStep,2).reached);assert.equal(C.analyze(noStep,2).settling_s,null);
const fitted3=recording();fitted3.ingest('DIAG=1 ENC_INSTALLED3=1,1,1 ATK_u3=11667,5000,8000');fitted3.ingest(frame(1,1,0,0).replace('ERPM3_m=0,0,120000','ERPM3_m=1200000,2400000,3600000'),100);
assert.deepEqual(fitted3.rows[0].installed,[1,1,1]);
assert.deepEqual(C.parseCsv(C.toCsv(fitted3))[0].rpm,[1200,2400,3600]);
console.log('PASS report: measured vs target, last-step metrics, 5° overshoot / 1.8 s observed settling, excursion/gap/invalid revocation, raw CSV roundtrip, units, uint32 wrap/reset, pause boundary and peak-preserving plot reduction');
