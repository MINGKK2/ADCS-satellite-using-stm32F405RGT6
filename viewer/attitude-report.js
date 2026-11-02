(function(){
  'use strict';
  const C=globalThis.AttitudeReportCore,$=id=>document.getElementById(id);
  if(!C||!$('reportChart'))return;
  const rec=new C.Recorder();
  const el=Object.fromEntries(['reportTitle','reportAxis','reportKind','reportWindow','reportTolerance','reportHold','reportStart','reportStop','reportClear','reportCsv','reportPng','reportSvg','reportJson','reportImport','reportFile','reportPasteText','reportPasteLoad','reportStatus','reportSource','reportChart','reportMetrics'].map(id=>[id,$(id)]));
  let dirty=true,plotBusy=false,lastPlot=0,exporting=false,revision=0,notice='',zoomRange=null;
  const names=['Roll','Pitch','Yaw'],colors=['#f4828e','#61d6a7','#78b4ff'];
  const fmt=(v,n=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(n):'—';
  const safe=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const selectedAxes=()=>el.reportAxis.value==='all'?[0,1,2]:[Number(el.reportAxis.value)];
  function options(){return {tolerance:Math.max(.01,Number(el.reportTolerance.value)||2),holdSeconds:Math.max(.1,Number(el.reportHold.value)||1)};}
  function visibleRows(){const sec=Number(el.reportWindow.value);let rows=rec.rows;if(sec&&rows.length){const start=rows[rows.length-1].t-sec;rows=rows.filter(r=>r.t>=start);}return zoomRange?rows.filter(r=>r.t>=zoomRange[0]&&r.t<=zoomRange[1]):rows;}
  function analysis(){const rows=visibleRows();return selectedAxes().map(a=>C.analyze(rows,a,options()));}
  function filename(ext){const title=el.reportTitle.value||rec.title;return title.normalize('NFD').replace(/[\u0300-\u036f]/g,'').replace(/đ/gi,'d').replace(/[^a-z0-9_-]+/gi,'_').slice(0,75)+'_'+new Date().toISOString().replace(/[:.]/g,'-')+'.'+ext;}
  function download(content,type,name){const url=URL.createObjectURL(new Blob([content],{type}));const a=document.createElement('a');a.href=url;a.download=name;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function setNotice(s){notice=s;dirty=true;}
  function uiState(){
    const n=rec.rows.length,valid=rec.rows.filter(r=>r.valid).length,last=rec.rows[n-1];
    const stalled=rec.recording&&(!last||performance.now()-last.hostMs>1000);
    el.reportSource.textContent=rec.source==='CSV'?'Nguồn: CSV nhập · '+rec.title:'Nguồn: UART STM32 · góc EKF';el.reportSource.className='report-source'+(rec.source==='CSV'?' csv':'');
    el.reportStatus.className='report-status'+(rec.recording&&!stalled?' recording':notice||stalled?' error':'');
    el.reportStatus.textContent=(notice||rec.message)+(n?' · '+n.toLocaleString('vi-VN')+' mẫu / '+valid.toLocaleString('vi-VN')+' hợp lệ · '+fmt(last.t,1)+' s':'')+(stalled?' · Chờ bản tin mới; không nội suy mẫu bị mất.':'');
    el.reportStart.disabled=rec.recording;el.reportStart.textContent=n&&rec.source==='UART'?'Ghi tiếp':'Bắt đầu ghi';el.reportStop.disabled=!rec.recording;
    for(const k of ['reportCsv','reportPng','reportSvg','reportJson'])el[k].disabled=!n||exporting;
    el.reportImport.disabled=rec.recording;el.reportPasteLoad.disabled=rec.recording;el.reportClear.disabled=rec.recording;
    const stats=analysis();el.reportMetrics.replaceChildren();
    for(const s of stats){const tr=document.createElement('tr');
      const state=s.reached?'Trong dải · '+fmt(s.observed_hold_s,1)+' s quan sát':s.valid?'Chưa giữ đủ dải sai số':'Chưa đủ dữ liệu hợp lệ';
      const vals=[s.axis.toUpperCase(),fmt(s.target)+'°',fmt(s.final_mean_error_deg)+'°',fmt(s.rmse_deg)+'°',fmt(s.overshoot_deg)+'°',s.settling_s===null?(s.valid&&!s.step_observed?'— (chưa có bước target)':'—'):fmt(s.settling_s)+' s',s.valid+'/'+s.total,state];
      for(const text of vals){const td=document.createElement('td');td.textContent=text;tr.appendChild(td);}tr.lastChild.style.color=s.reached?'#52e0a4':'#ffd166';el.reportMetrics.appendChild(tr);
    }
  }
  function chartSpec(light=false){
    const rows=visibleRows(),axes=selectedAxes(),kind=el.reportKind.value,tolerance=options().tolerance,data=[],annotations=[],shapes=[];
    const n=axes.length,fg=light?'#192738':'#dceaf7',grid=light?'#dbe2ea':'#22354a';
    const layout={paper_bgcolor:light?'white':'#091624',plot_bgcolor:light?'white':'#091624',font:{family:'Arial, sans-serif',color:fg,size:light?15:12},margin:{l:65,r:28,t:light?125:56,b:light?122:48},
      height:n===3?(light?1100:740):(light?850:460),hovermode:'x unified',showlegend:true,legend:{orientation:'h',x:0,y:light?1.12:1.12},
      uirevision:String(revision),annotations,shapes};
    if(light){const fw=[...new Set(rows.map(r=>r.fw).filter(Boolean))].join(', ')||'FW chưa có';
      layout.title={text:safe(el.reportTitle.value||rec.title),x:.04,y:.98,font:{size:23}};
      const count=rows.filter(r=>r.valid).length;
      annotations.push({xref:'paper',yref:'paper',x:0,y:-.13,xanchor:'left',yanchor:'top',showarrow:false,align:'left',font:{size:12,color:'#49566a'},text:safe((rec.source==='CSV'?'Nguồn: CSV nhập':'Nguồn: UART STM32')+' · '+fw+' · '+count+'/'+rows.length+' mẫu tư thế hợp lệ')+'<br>'+safe('Góc đo EKF; dữ liệu lỗi/mất đoạn không nối. Dải ±'+options().tolerance+'°, giữ ≥'+options().holdSeconds+' s; kết quả chỉ trong đoạn đã ghi.')});
    }
    const definitions={angle:{key:'angle',unit:'Góc (°)',label:'đo EKF'},error:{key:'error',unit:'Sai số Euler (°)',label:'target − đo'},pwm:{key:'pwm',unit:'PWM có dấu',label:'PWM thực'},rpm:{key:'rpm',unit:'Tốc độ bánh đà (RPM)',label:'RPM magnitude'},gyro:{key:'gyro',unit:'Tốc độ góc thân (°/s)',label:'gyro hệ thân'}};
    const def=definitions[kind];
    axes.forEach((axis,j)=>{
      const suffix=j?String(j+1):'',xref='x'+suffix,yref='y'+suffix,top=1-j/n,bottom=1-(j+1)/n+.11*(n>1?1:0);
      layout['xaxis'+suffix]={anchor:yref,domain:[0,1],title:j===n-1?{text:'Thời gian từ đầu phiên (s)'}:undefined,showticklabels:j===n-1,gridcolor:grid,zeroline:false,...(zoomRange?{range:zoomRange,autorange:false}:{autorange:true}),...(j?{matches:'x'}:{})};
      layout['yaxis'+suffix]={anchor:xref,domain:[Math.max(0,bottom),top],title:{text:def.unit},gridcolor:grid,zerolinecolor:grid,automargin:true};
      annotations.push({xref:'paper',yref:'paper',x:0,y:top+.025,xanchor:'left',showarrow:false,text:'<b>'+names[axis].toUpperCase()+'</b>',font:{size:light?15:12,color:fg}});
      const points=C.indices(rows,axis,def.key),x=[],y=[],tx=[],ty=[],badX=[],badY=[];
      let prev=null;
      for(const i of points){const r=rows[i],measurement=r[def.key][axis];
        if(r.gapBefore){x.push(r.t);y.push(null);}
        if(kind==='angle'&&prev&&Math.abs(r.angle[axis]-prev.angle[axis])>180){x.push(r.t);y.push(null);}
        let ok=r.valid&&Number.isFinite(measurement);
        if(kind==='rpm')ok=Number.isFinite(measurement)&&r.installed[axis]===1&&r.diagAge!==null&&r.diagAge<=2500;
        if(kind==='pwm')ok=Number.isFinite(measurement);
        x.push(r.t);y.push(ok?measurement:null);
        if(!r.valid&&Number.isFinite(r.target[axis])){badX.push(r.t);badY.push(r.target[axis]);}
        if(kind==='angle'){if(prev&&Math.abs(r.target[axis]-prev.target[axis])>180){tx.push(r.t);ty.push(null);}tx.push(r.t);ty.push(r.target[axis]);}
        prev=r;
      }
      const color=light?['#b72542','#087950','#1856a6'][axis]:colors[axis];
      data.push({x,y,type:'scatter',mode:'lines',name:names[axis]+' '+def.label,line:{color,width:light?2.4:2},connectgaps:false,xaxis:xref,yaxis:yref,hovertemplate:'%{x:.3f} s · %{y:.3f}<extra>'+names[axis]+'</extra>'});
      if(kind==='angle')data.push({x:tx,y:ty,type:'scatter',mode:'lines',name:names[axis]+' target',line:{color:light?'#606b7c':'#dae0e8',width:1.8,dash:'dash',shape:'hv'},connectgaps:false,xaxis:xref,yaxis:yref});
      if(kind==='angle'&&badX.length)data.push({x:badX,y:badY,type:'scatter',mode:'markers',name:names[axis]+' mẫu tư thế lỗi',marker:{color:'#cf3a4c',size:5,symbol:'x'},xaxis:xref,yaxis:yref});
      if(kind==='error')for(const sign of [-1,1])shapes.push({type:'line',xref:xref+' domain',yref,x0:0,x1:1,y0:sign*tolerance,y1:sign*tolerance,line:{color:light?'#a06e08':'#ffd166',width:1,dash:'dot'}});
      if(light){const stat=C.analyze(rows,axis,options());const detail='RMSE '+fmt(stat.rmse_deg)+'° · sai số cuối '+fmt(stat.final_mean_error_deg)+'° · vào dải '+(stat.settling_s===null?'chưa xác nhận':fmt(stat.settling_s)+' s');
        annotations.push({xref:'paper',yref:'paper',x:1,y:top+.025,xanchor:'right',showarrow:false,text:safe(detail),font:{size:n===3?11:12,color:'#49566a'}});}
    });
    if(!rows.length)annotations.push({xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false,text:'Nhấn Bắt đầu ghi, sau đó đặt góc mục tiêu.<br>Hoặc mở CSV đã ghi để vẽ lại.',font:{size:15,color:light?'#606b7c':'#8fa5bd'}});
    return {data,layout};
  }
  async function draw(force=false){
    if(!globalThis.Plotly){setNotice('Không tải được Plotly cục bộ; CSV vẫn có thể xuất.');uiState();return;}
    if(plotBusy||exporting||!force&&!dirty)return;plotBusy=true;dirty=false;
    try{const spec=chartSpec();el.reportChart.style.height=spec.layout.height+'px';await Plotly.react(el.reportChart,spec.data,spec.layout,{responsive:true,displaylogo:false,scrollZoom:false,modeBarButtons:[['zoom2d','pan2d'],['zoomIn2d','zoomOut2d','autoScale2d','resetScale2d']],doubleClick:'reset'});
      if(!el.reportChart.__reportZoomBound){el.reportChart.__reportZoomBound=true;el.reportChart.on('plotly_relayout',e=>{
        for(const k of ['xaxis','xaxis2','xaxis3']){
          if(e[k+'.autorange']===true){if(zoomRange){zoomRange=null;dirty=true;}break;}
          const range=e[k+'.range']??[e[k+'.range[0]'],e[k+'.range[1]']];
          if(range.every(Number.isFinite)&&range[1]>range[0]&&(!zoomRange||range.some((v,i)=>v!==zoomRange[i]))){zoomRange=[...range];dirty=true;break;}
        }
      });}
    }
    catch(e){setNotice('Không vẽ được biểu đồ: '+e.message);}finally{plotBusy=false;lastPlot=performance.now();}
  }
  async function exportImage(format){
    if(!rec.rows.length||exporting)return;
    if(!globalThis.Plotly){setNotice('Plotly chưa sẵn sàng.');return;}
    exporting=true;uiState();const host=document.createElement('div');host.className='report-export-host';document.body.appendChild(host);
    try{const spec=chartSpec(true);await Plotly.newPlot(host,spec.data,{...spec.layout,width:1600},{staticPlot:true,displayModeBar:false});
      const image=await Plotly.toImage(host,{format,width:1600,height:spec.layout.height,scale:format==='png'?2:1});
      // Blob downloads avoid large data-URL navigation limits for report PNGs.
      const at=image.indexOf(','),header=image.slice(0,at),payload=image.slice(at+1);
      const bytes=/;base64/i.test(header)?Uint8Array.from(atob(payload),c=>c.charCodeAt(0)):decodeURIComponent(payload);
      download(bytes,format==='svg'?'image/svg+xml;charset=utf-8':'image/png',filename(format));setNotice('Đã tạo file '+format.toUpperCase()+' nền trắng; hoàn tất lưu nếu trình duyệt hỏi.');
    }catch(e){setNotice('Lỗi xuất ảnh: '+e.message);}finally{Plotly.purge(host);host.remove();exporting=false;uiState();}
  }
  el.reportStart.addEventListener('click',()=>{try{notice='';rec.start(el.reportTitle.value);}catch(e){setNotice(e.message);}dirty=true;uiState();});
  el.reportStop.addEventListener('click',()=>{notice='';rec.stop();dirty=true;uiState();});
  el.reportClear.addEventListener('click',()=>{if(rec.rows.length&&!confirm('Tạo phiên mới sẽ xóa dữ liệu đang ghi trong trình duyệt. Bạn đã xuất CSV cần giữ chưa?'))return;rec.clear();notice='';zoomRange=null;revision++;dirty=true;uiState();draw(true);});
  for(const k of ['reportAxis','reportKind','reportWindow','reportTolerance','reportHold'])el[k].addEventListener('change',()=>{zoomRange=null;revision++;dirty=true;uiState();draw(true);});
  el.reportTitle.addEventListener('change',()=>{rec.title=el.reportTitle.value.slice(0,100);dirty=true;});
  el.reportCsv.addEventListener('click',()=>{download(C.toCsv(rec),'text/csv;charset=utf-8',filename('csv'));setNotice('Đã xuất toàn bộ mẫu của phiên, gồm cờ lỗi và thời gian MCU/PC.');});
  el.reportJson.addEventListener('click',()=>{const rows=visibleRows();download(JSON.stringify({title:el.reportTitle.value,source:rec.source,firmware:[...new Set(rows.map(r=>r.fw).filter(Boolean))],started_utc:rec.startedUtc,exported_utc:new Date().toISOString(),criteria:options(),window_seconds:Number(el.reportWindow.value)||null,zoom_range_s:zoomRange,total_session_samples:rec.rows.length,analyzed_samples:rows.length,analysis_scope:'Last constant-target segment per axis within selected window; settling time requires an observed target step; observed history only, not future stability or independent physical angle verification.',axes:analysis()},null,2),'application/json',filename('json'));});
  el.reportPng.addEventListener('click',()=>exportImage('png'));el.reportSvg.addEventListener('click',()=>exportImage('svg'));
  async function importText(text,name){
    if(rec.recording)throw Error('Dừng ghi UART trước khi mở CSV.');
    if(rec.rows.length&&!confirm('Mở CSV sẽ thay dữ liệu hiện tại. Tiếp tục?'))return;
    const rows=C.parseCsv(text);rec.load(rows,name);el.reportTitle.value=name.replace(/\.csv$/i,'');notice='';zoomRange=null;revision++;dirty=true;uiState();await draw(true);
  }
  el.reportPasteLoad.addEventListener('click',async()=>{try{await importText(el.reportPasteText.value,'CSV đã dán');}catch(e){setNotice('Không đọc CSV: '+e.message);}});
  el.reportImport.addEventListener('click',()=>el.reportFile.click());
  el.reportFile.addEventListener('change',async()=>{
    const file=el.reportFile.files?.[0];if(!file)return;
    try{if(file.size>24*1024*1024)throw Error('CSV vượt 24 MB.');await importText(await file.text(),file.name);
    }catch(e){setNotice('Không mở CSV: '+e.message);}finally{el.reportFile.value='';}
  });
  globalThis.attitudeReport={
    ingest(line){const wasRecording=rec.recording;const row=rec.ingest(line,performance.now());if(row||wasRecording!==rec.recording)dirty=true;},
    connectionLost(){if(rec.recording){rec.stop();rec.message='COM đã ngắt: dừng ghi và giữ dữ liệu để xuất.';}dirty=true;}
  };
  setInterval(()=>{uiState();if(performance.now()-lastPlot>=500)draw();},500);
  uiState();draw(true);
})();
