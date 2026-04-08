# Dive Computer

Based on the previous [bottom timer project]([https://](https://github.com/tl5915/Bottom-Timer-for-DPV)). Pressure sensor and housing remains the same.

- Add dive computer mode with Bühlmann ZHL-16C algorithm
- Add tilt compensated digital compass with soft and hard iron calibration
- Backlight auto adjustment

## Hardware

- Change MCU from ESP32-C3 to ESP32-S3 (Waveshare ESP32-S3-LCD-1.69)
- Change display from 1.54 inch OLED to 1.69 inch IPS LCD
- Onboard QMI8658 6-axis IMU
- Offboard QMC5883L magnetometer
- Offboard BH1750 light sensor

## Software

- Press onboard button to start compass calibration, rotate in all directions in 60 seconds
- Dive timer start at 1m depth
- Default bottom timer view with digital compass
- Triple tap to switch to dive computer mode:
  
  | Stop | Time | TTS | surfGF |
- In dive computer mode, double tap to change gradient factor to `100/100`

### Decompression model assumptions

- Fixed gradient factor `60/85` (optional overide)
- Close circuit with fixed set point of `1.2`
- Ascent rate 9 m/min
- Sea level 1 atm
- No prior dives
- No helium penalty

