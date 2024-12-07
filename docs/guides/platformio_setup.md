# PlatformIO Quick Start

## Installation
1. VSCode → Extensions (Ctrl+Shift+X)
2. Search "PlatformIO"
3. Install and Restart VSCode

## Project Setup
1. PlatformIO Home → New Project
  ### Inside "Project Wizard" select the following:
  - Name your: your_project
  - Name: your_project
  - Board: select "Espressif ESP32 Dev Module"
  - Framework: Arduino
  - Location: Either select default (/home/Documents/PlatformIO) or select your own folder
  - Click Finish
  ### Your project is created, you should now be in the "platformio.ini" folder
  ### Here you should add the following line for Baudrates after framework: 
  - monitor_speed = 115200

2. Project Structure:
  /src         → Main code (main.cpp)
  /include     → Header files
  /lib         → Project libraries
  platformio.ini → Config file

## Basic platformio.ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

## Key Commands
Build:    pio run
Upload:   pio run --target upload
Monitor:  pio device monitor
Clean:    pio run --target clean

## USB Access
If permission denied:
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyUSB0
EOL