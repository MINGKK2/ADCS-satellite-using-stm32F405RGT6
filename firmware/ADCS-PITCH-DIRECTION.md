# Pitch direction correction — built, not yet flashed

Firmware identifier: `AXIS_FRAME_RP_PITCH_V3`.

The attitude controller's Pitch motor polarity changed from -1 to +1 after the
operator reported that Pitch was rotating in the opposite direction. This
reverses the signed Pitch PWM command for the same attitude error and body rate.
Roll/Yaw polarity, sensor/body coordinate mapping and PID gains remain unchanged.

Pitch wiring is unchanged: PA7/TIM3_CH2 PWM, PB12 IN1, PB13 IN2, PC6 encoder.
The one-channel encoder still measures RPM magnitude; it cannot independently
verify rotation direction.

Full rebuild passed without compiler/linker warnings. Existing tests using the
actual PID/EKF/encoder functions and telemetry protocol passed. ELF load segments
end at 0x0801DDF4; calibration sector 11 remains reserved. Fresh BIN and HEX were
generated from the rebuilt ELF (the IDE makefile does not generate them itself).

Package, prior source/binaries and hashes:
`C:\Users\Admin\Documents\ChatGPT\HKVT\work\pitch-direction-fix-20261101-124612-UTC`.

This record describes a build only. The board has not been flashed or motor-tested
with this correction. Keep the actual active-firmware deployment record separate
until DFU download and verification are completed. No ESP changes are needed.
