# Active firmware — V7

AXIS_FRAME_RP_ENCSUP_V7 programmed via USB DFU; flash verified and application started.
Only sectors0–4 erased. Calibration sector11 full128KiB hashes before/after matched:
1c5e9607650a4b939c8686787e1420b1586c58b8511ba0a4efd223228ed86c57
Saved gyro/accel/compass calibration preserved (measured flags7, sequence7).
Post-flash COM23 radio telemetry confirms FW V7, FLASH/MEASURED7/SAVE0,
ATT_VALID1, SENSOR_ERR0x00, EF3=0,0,0, EREC3=0,0,0, PID_BLOCK3=0,0,0.
Two STATUS ACKs received; approximately18.67fast frames/s in12seconds,6sequence gaps.
PID OFF and PWM0 throughout. No motor-powered hardware test was performed.

V7 restarts no-pulse monitoring on commanded reversal/stop/sub-threshold PWM;
continuous same-direction no-pulse timeout, implausible-rate and RPM limits remain.
Clear requires operator command, all PID/outputsOFF and25 plausible recovery
pulses per faulted wheel collected while disarmed. No automatic clear or rearm.
HTML lists only faulted fitted axes and explains the actual encoder observation.

Gains unchanged Kp11.6666667 Ki5 Kd8; motor signs Roll+1/Pitch+1/Yaw-1.
All3 encoders installed: PA8/PC6/PA0,25rising-edge pulses/rev, magnitude only.
No EKF/calibration/GPIO/UART/.ioc/ESP configuration changes.

Motor-off manual tests before deployment: Roll approximately2turns +52/+1/0;
Pitch approximately2turns +0/+52/0. Original transient under motor drive is
not fully identified; reproduction of reversal timing defect is a host test.
No claim that the physical motor-on false-fault issue is conclusively eliminated.

Deployment: C:\Users\Admin\Documents\ChatGPT\HKVT\work\encoder-fault-fix-20261102-002237\dfu-deployment-20261101-175921-UTC\deployment.json
Build package: C:\Users\Admin\Documents\ChatGPT\HKVT\work\encoder-fault-fix-20261102-002237
