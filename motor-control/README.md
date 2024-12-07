# Motor Controller

ESP32 motor controller code using PlatformIO for remote building and flashing.

## Setup
- Install PlatformIO (VS Code extension)
- Clone this repo
- Open this folder in PlatformIO

## Why PlatformIO?
We use PlatformIO instead of Arduino IDE because it lets us:
1. Build on our own computer
2. SSH the binary to Raspberry Pi
3. Flash it to the ESP32 that's connected to the Pi via serial

## Files
- `/src/main.cpp` - Main code
- `platformio.ini` - Project config including:
   - Board config (esp32dev)
   - Upload settings (SSH to Pi + esptool commands)
   - Serial monitor config (through network socket)
   - Debug settings

The platformio.ini takes care of sending our compiled code to the Pi at 192.168.0.150 and using esptool to flash it to the ESP32 connected on /dev/ttyUSB0.
// note that the IP and /dev/ttyUSB0 are not nessesarly static and could change.