# Calibration verified — 2026-11-02 (Asia/Bangkok)

Firmware AXIS_FRAME_RP_ENC3_V6; no firmware changes or reflashing required.
Motor power was disconnected by the operator. All PID requests/output remained OFF/0.

Gyro recalibration succeeded while stationary. Accelerometer captured six distinct
sensor faces: -Z,+Z,+X,-X,+Y,-Y. One duplicate -Z capture was rejected correctly.
Accel offsets raw: [158.8,144.7,-201.6]; scales [1.000496,0.999844,0.990756].
Gyro was recalibrated automatically after the sixth face:
body-frame bias in microrad/s [50617,-18608,26344].
Compass was manually rotated through3D for20seconds. Min/max diagonal calibration:
raw sensor offsets [-3364,-6913,3273]; scales [.974348,1.006698,1.020067].
All three saves returned RESULT=0 SOURCE=FLASH MEASURED=7.

After operator power cycle, the log explicitly reports:
CAL STATUS TYPE=FUSION PHASE=READY CALIBRATION=STORED
Telemetry confirms CAL_SOURCE=FLASH CAL_MEASURED=7 CAL_SAVE=0.
No calibration command was sent after the power cycle.
Magnetometer TRACKING, MAG_REASON=NONE, accepted870/rejected0 at final sample,
field ratio .998, innovation .785degrees, SENSOR_ERR=0x00, ATT_VALID=1.
Last IMU age5ms. The postboot stationary window was13.243seconds:
Roll range .154deg, Pitch .084deg, Yaw .263deg (peak-to-peak, not absolute accuracy).
Median corrected gyro[-.014,-.017,-.001]deg/s. Motors were not energized.

The temporary innovation rejection after compass rotation cleared after stationary
startup with the saved coefficients. This does not establish immunity to motor-on
magnetic disturbances or long-duration drift. Startup initializes attitude and a
heading reference; it does not repeat the manual sensor calibration. Yaw zero is
relative to the startup reference and is not an absolute true-north measurement.

Flash storage is append-only CRC-checked sector11 at0x080E0000, commit programmed
last and record verified. Current firmware loads valid coefficients at startup.
Do not mass-erase sector11 when updating firmware. Recalibration is not required
on each normal restart; keep the calibrated sensor/mount configuration unchanged.

Raw acquisition and calibration events: C:\Users\Admin\Documents\ChatGPT\HKVT\work\calibration-session
Power-cycle raw log: C:\Users\Admin\Documents\ChatGPT\HKVT\work\axis-check\calib-powercycle-20261102.log
Verification JSON: C:\Users\Admin\Documents\ChatGPT\HKVT\work\calibration-session\powercycle-verification.json
Viewer reopened at http://127.0.0.1:8765/attitude-viewer.html; COM23 device
selection is pending in browser. All CLI ports are closed.
