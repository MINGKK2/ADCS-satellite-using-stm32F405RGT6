# Active firmware — V8

AXIS_FRAME_RP_ENCNOSTOP_V8 programmed via USB DFU and flash verified.
Application resumed and radio COM23 returned2STATUS ACKs and about18.95fastHz.
Actual DIAG reports ENC_NO_PULSE_STOP=0: missing/zero encoder pulses no longer
create a fault, stop PWM or clear the operator PID request. The timeout constants,
monitor timer/direction state and0x01 fault creation have been removed.
Encoder count/RPM diagnostics remain active. A disconnected encoder no longer
provides automatic motor shutdown. User explicitly requested this policy.

ESTOP, IMU/EKF validity, implausible pulse-rate fault0x02, PWM clamp and raw/filtered
RPM limits remain. This does not implement a signed RPM loop or change tuning.
Kp11.6666667 Ki5 Kd8; polarity+1/+1/-1 and encoder PA8/PC6/PA0 PPR25 unchanged.

All128KiB of calibration sector11 match before/after flash:
1c5e9607650a4b939c8686787e1420b1586c58b8511ba0a4efd223228ed86c57
Runtime confirms CAL_SOURCE=FLASH CAL_MEASURED=7 CAL_SAVE=0,
ATT_VALID1 SENSOR_ERR0x00 EF3=0,0,0 PID_BLOCK3=0,0,0.
PID OFF, PWM0 during verification. No powered motor/encoder-disconnect hardware
test was performed. Host actual-source tests cover all3axes60seconds without
pulses at +/-255, plus retained noise/overspeed and the complete regression suite.

HTML adds explicit no-pulse-stop state, marking it stale until fresh diagnostics.
Reload viewer and select COM23 to display the new field. All CLI ports closed.
Deployment: C:\Users\Admin\Documents\ChatGPT\HKVT\work\encoder-no-pulse-off-20261101-183908-UTC\dfu-deployment-20261101-184423-UTC\deployment.json
Build: C:\Users\Admin\Documents\ChatGPT\HKVT\work\encoder-no-pulse-off-20261101-183908-UTC
