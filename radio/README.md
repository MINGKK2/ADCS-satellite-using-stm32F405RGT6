# STM32F405 update through S3 → ESP-NOW → C3 → UART

This adds an explicit update mode to both bridges. Existing viewer commands and telemetry remain text lines at 115200 8N1. The STM32 ROM protocol runs locally on C3 at 115200 **8E1** using USART1 PA9/PA10; radio delay is not inserted between individual ROM-protocol bytes.

## Hardware and first installation

- STM32 PA9 TX → C3 GPIO20 RX; PA10 RX ← C3 GPIO21 TX; common GND, 3.3 V logic.
- S3 computer USB: COM23 on this machine. Do not let viewer/Serial Monitor hold the COM port while uploading.
- Install the new **C3 and S3 bridge firmware over their USB ports once**. Existing telemetry-only bridge firmware cannot install this feature over radio.
- Current configuration uses the STM32 B0/Reset buttons manually. No unverified GPIO was assigned to BOOT0/NRST. STM32 USB is not part of the wireless transfer.
- Keep motor power disconnected during bridge installation and STM32 programming. Staging also sends an explicit ESTOP.

Arduino ESP32 core used: 3.3.3. Build examples (Arduino CLI):

```powershell
arduino-cli compile --jobs 2 --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,FlashSize=4M,PartitionScheme=default,UploadSpeed=115200" --build-path build-c3 esp32c3_uart_bridge
arduino-cli compile --jobs 2 --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,UploadMode=default,FlashMode=qio,FlashSize=16M,PartitionScheme=fatflash,PSRAM=opi,UploadSpeed=115200" --build-path build-s3 esp32s3_ground_station
```

The S3 options match the previously built board. Confirm C3 chip/flash size and USB port before uploading; do not assume every COM port is the C3. Paired Wi-Fi addresses remain in the sketches; compare with the actual boards if changing hardware.

## Update procedure

Install host dependency: `python -m pip install -r requirements.txt`. Convert the linked ELF with `arm-none-eabi-objcopy -O binary firmware.elf firmware.bin`; the application remains linked at 0x08000000.

```powershell
python s3_updater.py --port COM23 --ping
python s3_updater.py --port COM23 --image firmware.bin --stage-only --motor-power-off
```

C3 receives the **whole image into its filesystem** before any STM32 erase. File size, CRC32, initial stack pointer and reset-vector address are checked. The filesystem is the C3 data partition, not STM32 Flash.

Then enter STM32 system bootloader with B0/Reset and release B0. Keep C3/S3 powered; do not reset C3 after staging because its in-RAM staging state would be lost. Run:

```powershell
python s3_updater.py --port COM23 --image firmware.bin --flash-staged --bootloader-ready --motor-power-off --log-dir deployment
```

The host verifies that the staged file matches, C3 probes PID 0x413 and 1024 KiB Flash, hashes all 128 KiB of calibration sector 11, erases only application sectors, writes/verifies every block, compares the calibration SHA-256 again, then issues GO. C3 restores UART 8N1. The host checks for fast telemetry and `FW=MANUAL_PID_V1`; it never arms PID.

The existing local DFU backup retains the actual calibration bytes. The wireless updater checks the entire sector before/after and does not upload raw calibration contents to Git.

## Protocol and failure behavior

- Radio requests/responses use new packet types 0x40/0x41 inside the existing CRC16 envelope.
- USB update line: `@FW <session> <request-id> <operation> <hex-payload-or-dash>`.
- Reply: `@FWACK <session> <request-id> <operation> <status> <hex-data>`; progress uses `@FWPROGRESS`.
- Stop-and-wait requests retry with **the same id and bytes**. C3 caches the last response and rejects old or changed-payload requests; a lost ACK does not append a chunk or erase Flash twice.
- ROM commands exposed: GET, GET ID, READ, WRITE, sector ERASE/EXTENDED ERASE and GO. No mass erase, readout unprotect, write unprotect, or option-byte writes.
- Maximum image 896 KiB, sector 11 never erased/written. Wrong vector, wrong MCU/Flash size, timeout, NACK, readback mismatch or calibration mismatch prevents GO.
- If power fails after erase, the old application may no longer run. The ST ROM bootloader remains the recovery path; enter B0/Reset and retry or use USB DFU. This is not a dual-bank atomic update.
- CRC/checksums detect transfer corruption; the paired-MAC ESP-NOW link is not cryptographic firmware authentication. This bench updater has no signed-image/secure-update mechanism.

## Tests and verification scope

`tests/test-rom.cpp` compiles the actual ROM driver with a scripted UART. It checks sync/parity, ID/size responses, ACK/NACK/timeouts, sector-list encoding, protected-sector/address/alignment guards, CRC/vector checks and duplicate-request handling. `tests/test-host.py` checks host retry identity, session matching and image guards.

Build success and these tests do not prove a wireless hardware transfer. Record an actual `deployment.json` only after readback and calibration comparison succeed; report separately whether post-GO telemetry and motor/attitude operation were tested.

## Radio issues verified during commissioning

Both peers use Wi-Fi channel 1, with Arduino `WiFi.setSleep(false)` set **before** `WiFi.mode(WIFI_STA)`. The 3.3.3 STA_START handler reapplies `WiFi.getSleep()`; a low-level `esp_wifi_set_ps(NONE)` alone was overwritten (measured PS=1). The retained policy now yields PS=0 on both.

On the connected C3, default/high TX power delivered no return ACK even though C3 received valid S3 requests. Limiting C3 to 34 quarter-dBm (8.5 dBm) changed the measured result to 2909 MAC ACKs / 0 failures and a successful bidirectional updater PING; RSSI was -38 dBm. This is a measured bench setting for this hardware, not a range guarantee.

C3 sends one packet at a time, paced by the ESP-NOW send callback. Main-task update replies have priority and bounded retries; normal telemetry cannot flood the TX pool. SDK logging is silenced on UART0, and bulk USB debug is off by default. USB diagnostics have zero TX timeout and report `C3STAT` every two seconds when the host can accept them.

S3 local diagnostic command `@FWSTAT` reports the actual MAC, channel, PS state and RX/TX counters. Updater control replies are separated by CRLF; the host also resynchronizes at the control marker if an older transparent fragment precedes it. No serial monitor may hold COM23 while the uploader runs.
