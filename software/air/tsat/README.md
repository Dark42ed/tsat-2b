# TSAT-2B Air

## Compilation Requirements

Compilation is recommended to be done in the Arduino IDE. Libraries may be installed through the library manager.

- [RFM69](https://github.com/LowPowerLab/RFM69) by LowPowerLab
- [Adafruit BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX) by Adafruit
- [Adafruit MMA8451](https://github.com/adafruit/Adafruit_MMA8451_Library) by Adafruit
- FS.h
- SD.h
- SPI.h

## Operation

1. Ensure that all components are properly wired to the ESP.
2. Insert a properly formatted (FAT32) SD card into the SD card reader.
3. Plug in battery power.
4. Wait for a solid blue light to appear on the ESP.

## Errors

If the ESP does not detect the SD card or RFM module, initialization will fail and
it will begin to blink an error code. A reset may fix some of these errors.

- 1 Blink: RFM module not found.
- 2/3 Blinks: SD card not found. Ensure the SD card is inserted and properly formatted.

If pressure readings or acceleration readings are zero, one of the sensors may have failed initializing.
These errors are not marked as critical, so the software will continue to function without these sensors.