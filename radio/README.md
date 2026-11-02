# C3/S3 radio-only telemetry bridge

Current bridge firmware: **RADIO_ONLY_V1**, Arduino ESP32 core 3.3.3.
This directory is the currently deployed telemetry bridge. The STM32 wireless
updater, its ROM driver and host uploader were removed from the current tree;
the historical implementation remains in Git history at commit `3405635`.
Use USB DFU to program STM32.

## Wiring

- STM32 PA9 / USART1 TX → C3 GPIO20 / UART0 RX.
- STM32 PA10 / USART1 RX ← C3 GPIO21 / UART0 TX.
- Common GND, 3.3 V logic, 115200 baud / 8N1 / no flow control.
- C3 native USB is local diagnostics (COM34 on the deployment PC).
- S3 USB-to-serial is the viewer port (COM23 on that PC).
- UART0 is the only telemetry source. Legacy ground-command mirroring to UART1
  is retained, but do not connect a second STM32 telemetry UART.

Paired STA MACs: C3 `B0:A6:04:40:C2:74`, S3 `DC:B4:D9:0B:64:3C`, channel 1.
Both set retained `WiFi.setSleep(false)` before STA startup, then PS=NONE.
C3 TX power is 34 quarter-dBm = 8.5 dBm, the setting measured to work on this
bench. This is not a general maximum-range recommendation. The link uses a
paired-MAC filter and CRC, not cryptographic authentication.

## Build

```powershell
arduino-cli compile --jobs 2 --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,FlashSize=4M,PartitionScheme=default,UploadSpeed=115200" --build-path radio/build-c3 radio/esp32c3_uart_bridge
arduino-cli compile --jobs 2 --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,UploadMode=default,FlashMode=qio,FlashSize=16M,PartitionScheme=fatflash,PSRAM=opi,UploadSpeed=115200" --build-path radio/build-s3 radio/esp32s3_ground_station
```

These options match the actual boards used. Verify chip/flash/port before
uploading; keep motor power off. Close applications holding the upload COM.
There is no wireless STM32 update command in these sketches.

## Data and recovery behavior

Existing version-1 212-byte packet layout, 12-byte header, payload up to 200,
CRC16-CCITT. Type 1 carries UART0 data; type 2 carries ground commands.
Send callbacks pace one packet in flight. C3 has an 8192-byte UART0 RX buffer,
SDK UART logging disabled, verbose USB mirroring disabled, and bounded USB debug
chunks. UART/radio forwarding does not wait for a debug reader.

S3 validates paired peer/CRC, assembles complete newline-delimited STM32 records
and writes USB in available-size chunks. Duplicate/reordered packets are rejected;
a missing fragment discards the partial line and resynchronizes at newline.
Overlong lines are discarded rather than emitted as misleading mixed records.
ACK/NACK/calibration records are prioritized between whole USB lines. Explicit
ESTOP supersedes unsent ordinary commands and is prioritized on C3 as well.

Send `@LINKSTAT` + newline to S3 for local `S3STAT` counters; this diagnostic is
not forwarded to STM32. Native C3 USB reports `C3STAT` periodically. Counters
include CRC errors, MAC ACK/failure, queue drops and fragment gaps. Compare
counter deltas; a C3 reset during S3 operation can leave a historical gap count.

## Host regression test

From repository root, with a C++17 compiler on PATH:

```powershell
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Iradio/tests radio/tests/test-radio.cpp -o radio/tests/test-radio.exe
.\radio\tests\test-radio.exe
```

This compiles the actual packet/line-assembly code. It tests CRC/layout, payload
fragment sizes 1..200, loss/duplicates/reordering, restart, sequence wrap and
oversized-line recovery. It does not simulate the RF or power supply.

## Hardware evidence and limitations

Both ESPs were flashed and their data hashes verified. Before STM32 V4 deployment,
S3 received 480 fast records in 25 seconds (~19.17 Hz), with two STM32 STATUS
ACKs, no fast-sequence gaps or new radio fragment gaps. Another 12-second capture
with COM34 closed still returned 230 fast records and two STATUS ACKs.
Final C3 diagnostics measured CH=1, PS=0, power=34 quarter-dBm, TXACK increasing
641→1253 with TXFAIL=0. Source-side STM32 latest-sample coalescing can still skip
fast sequence numbers; this is distinct from RF fragment loss.

Physically unplugging C3 USB while powering it from STM32 then caused zero radio
telemetry and failed MAC ACKs for 30 seconds. The operator later reported recovery
and the viewer confirmed fresh valid attitude/ACK/RPM, but the recovered C3 power
arrangement was not confirmed. LED illumination does not measure rail stability.
For a separate-power check, disconnect the STM32-to-C3 VCC feed before powering
C3 from a USB charger/powerbank; retain common GND and both UART wires.

After STM32 V4 was flashed, COM23 again returned only S3 local counters and no
STM32 STATUS ACK during the 12-second check. Therefore this publication does not
claim that the current end-to-end link or ADCS response has passed a final test.

## Latest STM32 V9 runtime check

The older V4 link-check failure above is historical. The V9 post-flash check on
2026-11-02 received 230 FAST records in 12 seconds (~19.14 Hz), two STM32 STATUS
ACKs, no FAST sequence gaps, valid IMU and saved calibration. Radio source was
unchanged. This verifies communication, not independent C3 power robustness or
closed-loop ADCS performance.
