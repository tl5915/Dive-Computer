# Bottom Timer / Dive Computer V2

Add compass display. Enable dive computer mode with Bühlmann zhl-16c algorithm.

## Hardware

- Change MCU from ESP32-C3 to ESP32-S3 (Waveshare ESP32-S3-LCD-1.69)
- Change display from 1.54 inch OLED to 1.69 inch IPS LCD
- Onboard QMI8658 6-axis IMU
- Offboard QMC5883L magnetometer
- Offboard BH1750 light sensor

## Software

- Press onboard button to start compass calibration, rotate in all directions in 30 seconds
- Dive timer start at 1m depth
- Default bottom timer view with digital compass
- Triple tap to switch to dive computer mode: stop, time, TTS
- In dive computer mode, double tap to change gradient factor to 99/99 (**Emergency Only**)

### Decompression model assumptions

- Fixed gradient factor 60/85
- Close circuit with fixed set point of 1.2
- Ascent rate 9 m/min
- Sea level 1atm
- No prior dive
- No helium penalty

