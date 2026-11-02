# All three encoders fitted — V6 deployed

Firmware AXIS_FRAME_RP_ENC3_V6 has been programmed and verified via USB DFU.
Calibration sector 11 is unchanged after a full-sector before/after comparison.

Operator confirmed OUT wiring: Roll PA8/TIM1_CH1, Pitch PC6/TIM8_CH1,
Yaw PA0/TIM2_CH1; all three provide 25 rising-edge pulses per revolution.
Existing timer inputs already count all three. Roll is now marked present too;
telemetry reports ENC_INSTALLED3=1,1,1 and the viewer can display all three.
GPIO, timer filtering, PPR and RPM calculation are unchanged. Fault/overspeed
protection remains active. Presence does not prove a functioning pulse signal.

Gains retained: Kp=11.6666667, Ki=5, Kd=8. Motor signs +1/+1/-1 unchanged.
One-channel encoders measure speed magnitude only; DIR_VALID remains 0.
This change does not introduce a cascade RPM PID or direction measurement.
Full build and ADCS/encoder, protocol, viewer/report tests passed.
No physical pulse-count or motor test was performed. COM23 was absent after
flashing, so radio runtime telemetry has not been verified for this deployment.

With motor power off, rotate each wheel separately by hand and check that
only its corresponding EC3 count changes: one full turn should add 25 pulses.
Check FW=AXIS_FRAME_RP_ENC3_V6, PPR=25 and ENC_INSTALLED3=1,1,1 on telemetry.
