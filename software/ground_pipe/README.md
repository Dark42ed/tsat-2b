# Ground Pipe

This runs on the ESP on the ground. It receives packets from the air and
prints out each packet on an individual line, with each byte being a
space-seperated decimal.

## Compilation Requirements

Compilation is recommended to be done in the Arduino IDE. Libraries may be installed through the library manager.

- [RFM69](https://github.com/LowPowerLab/RFM69) by LowPowerLab

## Operation

1. Connect an RFM to an ESP via PCB or breadboard.
2. Flash the software to the ESP.
3. Connect the ESP to your computer to listen for packets via serial.

It's recommended you run the `ground` visualization tool rather than listening to serial output.