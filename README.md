# gps-sensor

Small geolocated sensing datalogger board for movement sensing.

The design is based on the [Adafruit Feather M0 Adalogger](https://www.adafruit.com/product/2796) and the insights from [PaulZC](https://github.com/PaulZC).

## Main Hardware:

 - MCU: ATSAMD21G18 
 - GPS: uBlox SAM-M8Q 
 - Acc: ADXL345
 - Microphone Connections:
[Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H](https://www.adafruit.com/product/3421)
[Adafruit Silicon MEMS Microphone Breakout - SPW2430](https://www.adafruit.com/product/2716)

## Libraries

 - [Adafruit ADXL245](https://learn.adafruit.com/adxl345-digital-accelerometer/overview)
 - [TinyGPS+](https://github.com/mikalhart/TinyGPSPlus)
 - [Arduino Log](https://github.com/thijse/Arduino-Log)
 - [Time](https://github.com/PaulStoffregen/Time)
 - Arduino Sound

## Firmware
The sample Arduino code use the bootloader from the  [Adafruit M0](https://learn.adafruit.com/compiling-m0-atsamd21-bootloader/compile)

## TODO
Hardware
 - Improve GPS fix time
Software
 - Improve Microphone reading accuracy
 - Dual Microphone noise cancelling

> Written with [StackEdit](https://stackedit.io/).

<!--stackedit_data:
eyJoaXN0b3J5IjpbNjgyODkxMjA1LC0xOTU1OTUzNTUxXX0=
-->