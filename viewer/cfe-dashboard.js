'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const fieldIds = ['Temperature','Pressure','Roll','Pitch','Yaw','Voltage',
    'RelativeAltitude','Latitude','Longitude','RemainingEnergy'];
  let port = null, reader = null, readTask = null, disconnecting = false;
  let decoder = new CfeTelemetry.Decoder(), txChain = Promise.resolve();
  let count = 0, lost = 0, bytes = 0, lastAt = null, previous = null;
  let rateTimes = [], latest = null;

  const status = (text, kind = '') => {
    $('streamStatus').textContent = text;
    $('streamStatus').className = 'badge ' + kind;
  };
  function controls(connected) {
    $('commandControls').disabled = !connected;
    $('messageId').disabled = connected;
    $('byteOrder').disabled = connected;
    $('connect').textContent = connected ? 'Ngắt kết nối' : 'Kết nối UART';
  }
  function markPoseInvalid() {
    if (globalThis.cubesatAttitude) globalThis.cubesatAttitude.valid = false;
    globalThis.cubesatModel?.setValidity(false);
  }
  function clearReadings() {
    fieldIds.forEach(name => { $(name).textContent = '—'; });
    markPoseInvalid();
  }
  function freshness() {
    const now = performance.now();
    rateTimes = rateTimes.filter(t => now - t < 1000);
    $('packetRate').textContent = rateTimes.length + ' gói/s';
    if (lastAt === null) return;
    const age = now - lastAt;
    $('lastPacket').textContent = age < 1000 ? Math.round(age) + ' ms trước' : (age/1000).toFixed(1) + ' s trước';
    const stale = age > 1500;
    $('readings').classList.toggle('stale', stale);
    if (stale) {
      status(port ? 'Dữ liệu cũ · không có gói mới' : 'Đã ngắt kết nối', 'warning');
      markPoseInvalid();
    } else if (port) status('Đang nhận gói V14', 'live');
  }
  function renderPacket(packet) {
    // The wire has a 14-bit counter. A reboot/uptime wrap starts a new epoch.
    if (previous && packet.sequence === previous.sequence
        && packet.uptimeSeconds === previous.uptimeSeconds) return;
    if (previous && packet.uptimeSeconds >= previous.uptimeSeconds) {
      const delta = (packet.sequence - previous.sequence + 16384) % 16384;
      if (delta > 0 && delta < 8192) lost += delta - 1;
    }
    previous = packet; latest = packet;
    count++; lastAt = performance.now(); rateTimes.push(lastAt);
    for (const name of fieldIds) {
      const value = packet.values[name];
      $(name).textContent = value === null ? '—' : value.toFixed(
        name === 'Latitude' || name === 'Longitude' ? 6 : name === 'Voltage' ? 4 : 2);
      $(name).closest('.metric').classList.toggle('unavailable', value === null);
    }
    $('packetCount').textContent = count.toLocaleString('vi-VN');
    $('packetLoss').textContent = lost.toLocaleString('vi-VN');
    $('sequence').textContent = packet.sequence;
    $('uptime').textContent = packet.uptimeSeconds.toFixed(3) + ' s';
    $('messageSeen').textContent = '0x' + packet.messageId.toString(16).toUpperCase().padStart(4,'0');
    $('discarded').textContent = decoder.discarded.toLocaleString('vi-VN');
    const { Roll: roll, Pitch: pitch, Yaw: yaw } = packet.values;
    const valid = [roll,pitch,yaw].every(Number.isFinite);
    globalThis.cubesatAttitude = {roll,pitch,yaw,valid};
    globalThis.cubesatModel?.setAttitude(globalThis.cubesatAttitude);
    $('readings').classList.remove('stale');
    status('Đang nhận gói V14', 'live'); freshness();
  }
  function feedBytes(chunk) {
    bytes += chunk.byteLength;
    $('byteCount').textContent = bytes.toLocaleString('vi-VN') + ' B';
    for (const packet of decoder.push(chunk)) renderPacket(packet);
    $('discarded').textContent = decoder.discarded.toLocaleString('vi-VN');
  }
  async function readLoop() {
    try {
      reader = port.readable.getReader();
      while (!disconnecting) {
        const {value,done} = await reader.read();
        if (done) break;
        if (value) feedBytes(value);
      }
    } catch (error) {
      if (!disconnecting) $('connectionNote').textContent = 'Lỗi UART: ' + error.message;
    } finally {
      reader?.releaseLock(); reader = null;
      if (!disconnecting) void disconnect(true);
    }
  }
  async function disconnect(fromReader = false) {
    if (disconnecting) return;
    disconnecting = true;
    try {
      if (!fromReader) {
        await reader?.cancel();
        await readTask;
      }
      await txChain.catch(() => {});
      await port?.close();
    } catch (error) { $('connectionNote').textContent = 'Đóng UART: ' + error.message; }
    finally {
      port = null; readTask = null; decoder.reset();
      controls(false); clearReadings(); lastAt = null; rateTimes = [];
      $('packetRate').textContent = '0 gói/s';
      $('lastPacket').textContent = '—';
      status('Chưa kết nối'); disconnecting = false;
    }
  }
  async function connect() {
    if (port) return disconnect();
    if (!navigator.serial) return status('Cần Chrome hoặc Edge và localhost', 'warning');
    const input = $('messageId').value.trim();
    if (!/^(?:0x)?[0-9a-f]{1,4}$/i.test(input)) return status('Message ID không hợp lệ', 'warning');
    try {
      decoder = new CfeTelemetry.Decoder({ messageId: parseInt(input.replace(/^0x/i,''),16),
        littleEndian: $('byteOrder').value === 'little' });
    } catch (error) { return status(error.message, 'warning'); }
    $('connect').disabled = true;
    try {
      port = await navigator.serial.requestPort();
      await port.open({baudRate:115200,dataBits:8,stopBits:1,parity:'none',flowControl:'none'});
      count = lost = bytes = 0; lastAt = null; previous = latest = null; rateTimes = [];
      clearReadings();
      for (const id of ['packetCount','packetLoss','sequence','discarded']) $(id).textContent = '0';
      $('byteCount').textContent = '0 B'; $('uptime').textContent = '—'; $('messageSeen').textContent = '—';
      $('commandStatus').textContent = 'Chưa gửi lệnh trong phiên này.';
      controls(true); status('Đã mở UART · chờ gói V14', 'warning');
      $('connectionNote').textContent = '115200 baud · 8N1 · 52 byte/gói. Đường truyền phải giữ nguyên byte nhị phân.';
      readTask = readLoop();
    } catch (error) {
      port = null; controls(false);
      status(error.name === 'NotFoundError' ? 'Đã hủy chọn cổng' : 'Không mở được UART', 'warning');
      $('connectionNote').textContent = error.message;
    } finally { $('connect').disabled = false; }
  }
  function sendCommand(command) {
    txChain = txChain.catch(() => {}).then(async () => {
      if (!port?.writable) throw new Error('Chưa kết nối UART');
      const writer = port.writable.getWriter();
      try {
        await writer.write(new TextEncoder().encode(command + '\r\n'));
        $('commandStatus').textContent = 'Đã gửi: ' + command + ' · chưa có phản hồi xác nhận từ STM32.';
      } finally { writer.releaseLock(); }
    }).catch(error => { $('commandStatus').textContent = 'Gửi lệnh thất bại: ' + error.message; });
    return txChain;
  }
  $('targetForm').addEventListener('submit', event => {
    event.preventDefault();
    const values = ['targetRoll','targetPitch','targetYaw'].map(id => Number($(id).value));
    if (['targetRoll','targetPitch','targetYaw'].some(id => $(id).value.trim() === '')
        || !values.every(Number.isFinite) || Math.abs(values[0]) > 180
        || Math.abs(values[1]) > 89 || Math.abs(values[2]) > 180) {
      $('commandStatus').textContent = 'Roll/Yaw trong ±180°, Pitch trong ±89°; nhập đủ ba góc.';
      return;
    }
    sendCommand('SET_ATTITUDE,' + values.join(','));
  });
  $('pidForm').addEventListener('submit', event => {
    event.preventDefault(); sendCommand('PID,' + $('pidAxis').value + ',' + $('pidAction').value);
  });
  document.querySelectorAll('[data-command]').forEach(button => {
    button.addEventListener('click', () => sendCommand(button.dataset.command));
  });
  $('connect').addEventListener('click', connect);
  navigator.serial?.addEventListener('disconnect', event => {
    if (event.target === port || event.port === port) void disconnect();
  });
  controls(false); clearReadings();
  if (!navigator.serial) $('connectionNote').textContent = 'Web Serial cần Chrome/Edge trên máy tính và trang HTTPS hoặc localhost.';
  setInterval(freshness, 250);
})();
