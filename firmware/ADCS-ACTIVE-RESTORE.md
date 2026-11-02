# Active firmware V9

`AXIS_FRAME_RP_DYN_V9` is the current source and verified USB DFU image.
Calibration sector 11 was preserved; S3 telemetry confirms FLASH mask 7/save 0,
valid IMU and no sensor error. Runtime UART/STATUS round trip is verified.
Motor-off attitude captures do not establish final ADCS stability or absolute
accuracy. See [current release notes](../docs/ADCS-V9-20261102.md),
[binary manifest](../artifacts/stm32-v9/manifest.json) and root README.
