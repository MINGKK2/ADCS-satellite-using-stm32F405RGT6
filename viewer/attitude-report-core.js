/* Recording/analysis only. No serial writer and no motor-control commands. */
(function(root,factory){
  const api=factory();
  if(typeof module==='object'&&module.exports)module.exports=api;
  else root.AttitudeReportCore=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
  'use strict';
  const AXES=['roll','pitch','yaw'],MAX_SAMPLES=36000;
  const wrap=v=>((v+180)%360+360)%360-180;
  const finite=v=>typeof v==='number'&&Number.isFinite(v);
  function tokens(line){return Object.fromEntries(line.trim().split(/\s+/).filter(t=>t.includes('=')).map(t=>{const i=t.indexOf('=');return [t.slice(0,i),t.slice(i+1)];}));}
  function number(v){return typeof v==='string'&&/^-?\d+(?:\.\d+)?$/.test(v)&&Number.isFinite(Number(v))?Number(v):null;}
  function vec(v,n=3,scale=1){if(typeof v!=='string')return Array(n).fill(null);const a=v.split(',').map(number);return a.length===n?a.map(x=>x===null?null:x/scale):Array(n).fill(null);}
  const allFinite=a=>Array.isArray(a)&&a.length===3&&a.every(finite);
  class Recorder{
    constructor(){this.rows=[];this.recording=false;this.meta={};this.lastSeq=null;this.lastBoard=null;this.clock=0;this.pendingGap=false;this.message='Chưa ghi dữ liệu.';this.source='UART';this.title='Đáp ứng tư thế CubeSat';this.startedUtc='';this.origin=null;this.hostOrigin=null;}
    start(title){if(this.requiresNewSession||this.source!=='UART'&&this.rows.length)throw Error('Xuất dữ liệu rồi tạo phiên mới trước khi ghi UART.');this.title=String(title||this.title).slice(0,100);this.source='UART';this.recording=true;this.pendingGap=this.rows.length>0;this.startedUtc=this.startedUtc||new Date().toISOString();this.message='Đang ghi dữ liệu UART thực.';}
    stop(){this.recording=false;this.message='Đã dừng ghi; dữ liệu được giữ để xuất báo cáo.';}
    clear(){this.rows=[];this.recording=false;this.requiresNewSession=false;this.lastSeq=null;this.lastBoard=null;this.clock=0;this.origin=null;this.hostOrigin=null;this.startedUtc='';this.pendingGap=false;this.source='UART';this.message='Phiên mới; nhấn Bắt đầu ghi trước khi gửi target.';}
    ingest(line,hostMs=0,utc=new Date().toISOString()){
      const f=tokens(line);
      if(!line.startsWith('SEQ=')){
        for(const k of ['FW','ATK_u3','ENC_INSTALLED3','MAG_STATE','ATT_MODE','CAL_SOURCE'])if(f[k]!==undefined)this.meta[k]={value:f[k],hostMs};
        return null;
      }
      const seq=number(f.SEQ),board=number(f.T_ms);
      if(seq===null||board===null||seq<0||seq>4294967295||board<0||board>4294967295)return null;
      let delta=0,seqDelta=1;
      if(this.lastBoard!==null){
        delta=(board-this.lastBoard)>>>0;
        if(delta>=0x80000000){
          this.lastBoard=board;this.lastSeq=seq;this.clock=0;this.meta={};
          if(this.recording){this.recording=false;this.message='STM32 khởi động lại: đã dừng phiên ghi. Xuất dữ liệu rồi tạo phiên mới.';}
          this.requiresNewSession=this.rows.length>0;this.pendingGap=true;return null;
        }
        seqDelta=(seq-this.lastSeq)>>>0;
        if(seqDelta===0||seqDelta>=0x80000000)return null;
      }
      this.clock+=delta;this.lastBoard=board;this.lastSeq=seq;
      if(!this.recording)return null;
      if(this.rows.length>=MAX_SAMPLES){this.stop();this.message='Đã đạt 36.000 mẫu; phiên ghi đã dừng. Hãy xuất dữ liệu.';return null;}
      if(this.origin===null){this.origin=this.clock;this.hostOrigin=hostMs;}
      const angle=['R','P','Y'].map(k=>number(f[k])).map(v=>v===null?null:v/1000);
      const target=vec(f.TARGET_mdeg,3,1000),age=number(f.IMU_AGE_ms);
      const valid=number(f.ATT_VALID)===1&&age!==null&&age>=0&&age<=20&&allFinite(angle)&&allFinite(target)&&Math.abs(angle[0])<=180&&Math.abs(angle[1])<=90&&Math.abs(angle[2])<=180;
      const previous=this.rows[this.rows.length-1];
      const gapBefore=!!previous&&(this.pendingGap||seqDelta>1||delta>250||hostMs-previous.hostMs>500);
      const m=k=>this.meta[k]?.value??'';
      const row={t:(this.clock-this.origin)/1000,hostMs,hostElapsed:(hostMs-this.hostOrigin)/1000,utc,board,seq,angle,target,
        error:angle.map((a,i)=>finite(a)&&finite(target[i])?wrap(target[i]-a):null),
        bodyError:vec(f.BERR_mdeg,3,1000),gyro:vec(f.G_mdps,3,1000),pwm:vec(f.PWM3),requested:vec(f.REQPWM3),pidOutput:vec(f.PIDOUT3),rpm:vec(f.ERPM3_m,3,1000),
        pid:vec(f.PID),active:vec(f.PID_ACTIVE),blocked:vec(f.PID_BLOCK3),estop:number(f.ESTOP),age,valid,gapBefore,
        reason:valid?'':(age===null||age>20?'IMU_STALE_OR_MISSING':'ATTITUDE_OR_TARGET_INVALID'),
        fw:m('FW'),gains:vec(m('ATK_u3'),3,1000),installed:vec(m('ENC_INSTALLED3')),
        diagAge:this.meta.ENC_INSTALLED3?hostMs-this.meta.ENC_INSTALLED3.hostMs:null,
        magState:m('MAG_STATE'),magAge:this.meta.MAG_STATE?hostMs-this.meta.MAG_STATE.hostMs:null};
      this.pendingGap=false;this.rows.push(row);return row;
    }
    load(rows,title,startedUtc=''){
      this.stop();this.rows=rows;this.title=String(title||'Dữ liệu CSV').slice(0,100);this.startedUtc=startedUtc;this.source='CSV';
      this.message='Dữ liệu nhập CSV; không phải phiên UART trực tiếp.';this.origin=null;this.hostOrigin=null;
    }
  }
  function analyze(rows,axis,{tolerance=2,holdSeconds=1}={}){
    if(!finite(tolerance)||tolerance<=0||!finite(holdSeconds)||holdSeconds<0.1)throw Error('Dải sai số và thời gian giữ phải dương.');
    const result={axis:AXES[axis],total:rows.length,valid:0,invalid:0,gaps:0,target:null,start_s:null,end_s:null,rmse_deg:null,final_mean_error_deg:null,overshoot_deg:null,settling_s:null,observed_hold_s:0,reached:false,step_observed:false};
    if(!rows.length)return result;
    // Analyze only the last constant-target segment. A future target cannot
    // retrospectively change the errors or metrics of an earlier command.
    let start=0,known=null,previousTarget=null;
    rows.forEach((r,i)=>{const t=r.target[axis];if(!finite(t))return;if(known!==null&&Math.abs(wrap(t-known))>0.01){start=i;previousTarget=known;result.step_observed=true;}known=t;});
    const slice=rows.slice(start);result.total=slice.length;result.target=known;result.start_s=slice[0].t;result.end_s=slice[slice.length-1].t;
    const usable=r=>r.valid&&finite(r.angle[axis])&&finite(r.target[axis])&&known!==null&&Math.abs(wrap(r.target[axis]-known))<=0.01;
    const good=slice.filter(usable);result.valid=good.length;result.invalid=slice.length-good.length;result.gaps=slice.filter(r=>r.gapBefore).length;
    if(!good.length)return result;
    const errors=good.map(r=>wrap(r.target[axis]-r.angle[axis]));
    result.rmse_deg=Math.sqrt(errors.reduce((s,e)=>s+e*e,0)/errors.length);
    const tail=good.filter(r=>r.t>=result.end_s-1);
    if(tail.length){const e=tail.map(r=>wrap(r.target[axis]-r.angle[axis])*Math.PI/180);result.final_mean_error_deg=Math.atan2(e.reduce((s,v)=>s+Math.sin(v),0),e.reduce((s,v)=>s+Math.cos(v),0))*180/Math.PI;}
    const direction=Math.sign(wrap(known-(previousTarget??good[0].angle[axis])));
    if(direction)result.overshoot_deg=Math.max(0,...good.map(r=>direction*wrap(r.angle[axis]-known)));
    // A gap, invalid point or later excursion revokes a previous settling claim.
    let first=slice.length;
    for(let i=slice.length-1;i>=0;i--){
      const r=slice[i];if(!usable(r)||Math.abs(wrap(r.target[axis]-r.angle[axis]))>tolerance)break;
      first=i;
      if(r.gapBefore||i>0&&r.t-slice[i-1].t>0.25)break;
    }
    if(first<slice.length){result.observed_hold_s=result.end_s-slice[first].t;
      if(result.observed_hold_s+1e-9>=holdSeconds){result.reached=true;if(result.step_observed)result.settling_s=slice[first].t-result.start_s;}}
    return result;
  }
  function csvCell(v){if(v===null||v===undefined)return '';if(typeof v==='number')return finite(v)?String(v):'';let s=String(v);if(/^[=+\-@\t\r]/.test(s))s="'"+s;return /[",\r\n]/.test(s)?'"'+s.replace(/"/g,'""')+'"':s;}
  const cols=['elapsed_s','pc_elapsed_s','pc_received_utc','board_uptime_ms','sequence','valid','imu_age_ms','gap_before','invalid_reason','estop','firmware','mag_state','mag_state_age_ms','diagnostic_age_ms','kp','ki','kd'];
  const vectorCols=[['angle','deg'],['target','deg'],['error','deg'],['bodyError','deg'],['gyro','deg_s'],['pwm','count'],['requested','count'],['pidOutput','count'],['rpm','rpm'],['pid','enabled'],['active','enabled'],['blocked','mask'],['installed','present']];
  const HEADERS=cols.concat(vectorCols.flatMap(([k,u])=>AXES.map(a=>k+'_'+a+'_'+u)));
  function toCsv(rec){
    const lines=[HEADERS.join(',')];
    for(const r of rec.rows){const vals=[r.t,r.hostElapsed,r.utc,r.board,r.seq,r.valid?1:0,r.age,r.gapBefore?1:0,r.reason,r.estop,r.fw,r.magState,r.magAge,r.diagAge,...r.gains];
      for(const [k]of vectorCols)vals.push(...r[k]);lines.push(vals.map(csvCell).join(','));}
    return '\uFEFF'+lines.join('\r\n')+'\r\n';
  }
  function parseCsv(text){
    if(text.length>24*1024*1024)throw Error('CSV quá lớn (tối đa 24 MB).');
    const matrix=[];let row=[],field='',quote=false;
    text=text.replace(/^\uFEFF/,'');
    for(let i=0;i<text.length;i++){
      const c=text[i];
      if(c==='"'){if(quote&&text[i+1]==='"'){field+='"';i++;}else if(!quote&&field)throw Error('Dấu nháy CSV không hợp lệ.');else quote=!quote;}
      else if(!quote&&(c===','||c==='\n')){row.push(field);field='';if(c==='\n'){matrix.push(row);row=[];if(matrix.length>MAX_SAMPLES+1)throw Error('CSV vượt 36.000 mẫu.');}}
      else if(c!=='\r'||quote)field+=c;
    }
    if(quote)throw Error('CSV chưa đóng dấu nháy.');
    if(field||row.length){row.push(field);matrix.push(row);}
    const header=matrix.shift();if(!header||header.length!==HEADERS.length||header.some((h,i)=>h!==HEADERS[i]))throw Error('CSV cần đúng định dạng xuất của mục biểu đồ này.');
    let prev=null;
    const rows=matrix.filter(r=>r.some(Boolean)).map((cells,i)=>{
      if(cells.length!==HEADERS.length)throw Error('Sai số cột CSV ở dòng '+(i+2));
      const f=Object.fromEntries(header.map((h,j)=>[h,cells[j]]));
      const n=k=>number(f[k]);const v=k=>AXES.map(a=>n(k+'_'+a+'_'+vectorCols.find(x=>x[0]===k)[1]));
      const r={t:n('elapsed_s'),hostElapsed:n('pc_elapsed_s'),utc:f.pc_received_utc,board:n('board_uptime_ms'),seq:n('sequence'),age:n('imu_age_ms'),estop:n('estop'),fw:f.firmware,magState:f.mag_state,magAge:n('mag_state_age_ms'),diagAge:n('diagnostic_age_ms'),reason:f.invalid_reason,gains:[n('kp'),n('ki'),n('kd')]};
      r.hostMs=finite(r.hostElapsed)?r.hostElapsed*1000:r.t*1000;
      for(const [k]of vectorCols)r[k]=v(k);
      r.error=r.angle.map((a,j)=>finite(a)&&finite(r.target[j])?wrap(r.target[j]-a):null);
      if(!finite(r.t)||r.t<0||prev&&r.t<=prev.t)throw Error('Thời gian CSV phải tăng, lỗi dòng '+(i+2));
      r.valid=n('valid')===1&&r.age!==null&&r.age>=0&&r.age<=20&&allFinite(r.angle)&&allFinite(r.target)&&Math.abs(r.angle[0])<=180&&Math.abs(r.angle[1])<=90&&Math.abs(r.angle[2])<=180;
      r.gapBefore=!!prev&&(n('gap_before')===1||r.t-prev.t>0.25||finite(prev.seq)&&finite(r.seq)&&((r.seq-prev.seq)>>>0)!==1);
      if(!r.valid&&!r.reason)r.reason='INVALID_IMPORTED_SAMPLE';prev=r;return r;
    });
    if(!rows.length)throw Error('CSV không có mẫu.');return rows;
  }
  function indices(rows,axis,key,max=2000){
    if(rows.length<=max)return rows.map((_,i)=>i);
    // Preserve extrema, gaps and target changes; metrics/CSV always use all rows.
    const keep=new Set([0,rows.length-1]),width=Math.ceil(rows.length/(max/4));
    for(let i=0;i<rows.length;i+=width){let lo=i,hi=i;
      for(let j=i;j<Math.min(i+width,rows.length);j++){
        const value=rows[j][key]?.[axis];
        if(finite(value)&&(!finite(rows[lo][key]?.[axis])||value<rows[lo][key][axis]))lo=j;
        if(finite(value)&&(!finite(rows[hi][key]?.[axis])||value>rows[hi][key][axis]))hi=j;
        if(j&&(rows[j].valid!==rows[j-1].valid||rows[j].gapBefore||Math.abs(wrap(rows[j].target[axis]-rows[j-1].target[axis]))>0.01)){keep.add(j-1);keep.add(j);}
      }
      keep.add(i);keep.add(Math.min(i+width-1,rows.length-1));keep.add(lo);keep.add(hi);
    }
    return [...keep].sort((a,b)=>a-b);
  }
  return {Recorder,analyze,toCsv,parseCsv,indices,wrap,AXES,HEADERS,MAX_SAMPLES};
});
