const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const http=require('node:http');
const {chromium}=require('playwright');
const {packet}=require('./test-cfe-telemetry.cjs');
const output=process.env.ADCS_UI_SCREENSHOT_DIR || path.join(require('node:os').tmpdir(),'adcs-cfe-ui');
fs.mkdirSync(output,{recursive:true});
const server=http.createServer((req,res)=>{
  const p=path.resolve(__dirname,'.'+decodeURIComponent(new URL(req.url,'http://localhost').pathname));
  if(!p.startsWith(__dirname+path.sep)){res.writeHead(403);return res.end();}
  fs.readFile(p,(error,data)=>{
    if(error){res.writeHead(404);return res.end();}
    const mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.json':'application/json','.bin':'application/octet-stream'};
    res.setHeader('Content-Type',mime[path.extname(p)]||'application/octet-stream');res.end(data);
  });
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser;
  try {
    try {browser=await chromium.launch({headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});}
    catch {browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});}
    const page=await browser.newPage({viewport:{width:1440,height:1100}});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.addInitScript(()=>{
      let controller;
      const serial=new EventTarget();
      const fake={
        readable:new ReadableStream({start(c){controller=c;}}),
        writable:new WritableStream({write(chunk){window.__uart.writes.push(Array.from(chunk));}}),
        async open(options){window.__uart.options=options;},
        async close(){window.__uart.closed=true;}
      };
      serial.requestPort=async()=>fake;
      Object.defineProperty(navigator,'serial',{configurable:true,value:serial});
      window.__uart={writes:[],feed(bytes){controller.enqueue(Uint8Array.from(bytes));},
        unplug(){const e=new Event('disconnect');Object.defineProperty(e,'port',{value:fake});serial.dispatchEvent(e);}};
    });
    await page.goto(`http://127.0.0.1:${server.address().port}/attitude-viewer.html`);
    await page.waitForFunction(()=>window.CfeTelemetry && document.getElementById('connect'));
    assert.equal(await page.locator('#Temperature').textContent(),'—');
    assert.equal(await page.locator('#targetForm button').isDisabled(),true);
    await page.locator('#connect').click();
    await page.waitForFunction(()=>!document.getElementById('commandControls').disabled);
    assert.equal(await page.evaluate(()=>__uart.options.baudRate),115200);
    const first=packet(10);
    await page.evaluate(b=>__uart.feed(b),Array.from(first.subarray(0,7)));
    assert.equal(await page.locator('#packetCount').textContent(),'0');
    await page.evaluate(b=>__uart.feed(b),Array.from(first.subarray(7)));
    await page.waitForFunction(()=>document.getElementById('packetCount').textContent==='1');
    for(const [id,value] of Object.entries({Temperature:'25.95',Pressure:'101325.00',Roll:'1.00',Pitch:'-2.00',Yaw:'169.00',Voltage:'1.2345',RelativeAltitude:'125.50',Latitude:'21.028833',Longitude:'105.851997',RemainingEnergy:'12.40'}))
      assert.equal(await page.locator('#'+id).textContent(),value,id);
    assert.equal(await page.evaluate(()=>cubesatAttitude.valid),true);
    await page.waitForFunction(()=>!document.getElementById('modelShell').disabled,{},{timeout:20000});
    await page.locator('#targetRoll').fill('10');await page.locator('#targetPitch').fill('-5');await page.locator('#targetYaw').fill('90');
    await page.locator('#targetForm button').click();
    for(const axis of ['ROLL','PITCH','YAW','ALL']){
      await page.locator('#pidAxis').selectOption(axis);await page.locator('#pidAction').selectOption('1');
      await page.locator('#pidForm button').click();
    }
    const commands=['ZERO_YAW','MODE,WORKING','MODE,CHARGING','ESTOP','ESTOP_CLEAR',
      'CAL_GYRO','CAL_MAG','CAL_ACCEL_START','CAL_ACCEL_CAPTURE','CAL_ACCEL_CANCEL'];
    for(const c of commands)await page.locator(`[data-command="${c}"]`).click();
    await page.waitForFunction(()=>__uart.writes.length===15);
    const writes=await page.evaluate(()=>__uart.writes.map(b=>new TextDecoder().decode(Uint8Array.from(b))));
    assert.deepEqual(writes,['SET_ATTITUDE,10,-5,90\r\n',...['ROLL','PITCH','YAW','ALL'].map(a=>`PID,${a},1\r\n`),...commands.map(c=>c+'\r\n')]);
    assert.match(await page.locator('#commandStatus').textContent(),/chưa có phản hồi/);
    await page.evaluate(b=>__uart.feed(b),Array.from(packet(12)));
    await page.waitForFunction(()=>document.getElementById('packetLoss').textContent==='1');
    await page.screenshot({path:path.join(output,'desktop.png'),fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.screenshot({path:path.join(output,'mobile.png'),fullPage:true});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true,'mobile horizontal overflow');
    await page.evaluate(b=>__uart.feed(b),Array.from(packet(13,{values:[NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN]})));
    await page.waitForFunction(()=>document.getElementById('Temperature').textContent==='—');
    assert.equal(await page.evaluate(()=>cubesatAttitude.valid),false);
    await page.waitForTimeout(1650);
    assert.match(await page.locator('#streamStatus').textContent(),/Dữ liệu cũ/);
    await page.evaluate(()=>__uart.unplug());
    await page.waitForFunction(()=>__uart.closed===true);
    assert.equal(await page.locator('#targetForm button').isDisabled(),true);
    await page.locator('#messageId').fill('oops');await page.locator('#connect').click();
    assert.match(await page.locator('#streamStatus').textContent(),/không hợp lệ/);
    assert.deepEqual(errors,[]);
    console.log('PASS browser: raw UART bytes, all ten fields, CAD pose, all command families, no fake ACK, loss count, stale/NaN, unplug cleanup, invalid config, mobile layout');
    console.log('Screenshots: '+output);
  } finally {await browser?.close();await new Promise(resolve=>server.close(resolve));}
})().catch(e=>{console.error(e);process.exitCode=1;});
