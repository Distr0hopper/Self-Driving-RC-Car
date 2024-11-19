# Safety Switch System

A wireless safety switch system using two ESP32s to create an emergency stop mechanism for the robot. When activated, it cuts power to the motor drivers through relays.

## Hardware Components

### Remote Control (Transmitter)
- **Board**: ESP32 Huzzah with LiPo battery support
- **Pin Configuration**:
  - Switch: Pin 26 (INPUT_PULLUP) - The physical safety switch
  - Battery Monitor Enable: Pin 13 - Enables voltage monitoring for LiPo
  - Battery Voltage Reading: Pin 35 - Reads LiPo battery voltage level

### Onboard Receiver
- **Board**: ESP32 Dev Module C
- **Pin Configuration**:
  - BLUE_LED: Pin 13 - Indicates active safety switch (ON when system is enabled)
  - GREEN_LED: Pin 26 - Indicates good wireless connection with remote
  - RED_LED: Pin 14 - Warning indicator (ON when connection lost)
  - RELAY_IN1: Pin 17 - Controls first relay for motor power
  - RELAY_IN2: Pin 18 - Controls second relay for redundancy

## Status Indicators
1. **Normal Operation**
   - Green LED ON = Remote connection good
   - Blue LED ON = Safety switch is enabled (system powered)
   - Red LED OFF = Everything normal

2. **Connection Lost**
   - Red LED ON = No signal from remote
   - Green LED OFF = Connection lost
   - Relays automatically OPEN = Motors disabled for safety

3. **Safety Switch OFF**
   - Blue LED OFF = System disabled
   - Relays OPEN = Motors cannot run

## Failsafe Features
- System requires constant communication (every 100ms)
- Auto-shutdown if signal lost for >500ms
- Dual relay system for redundant safety
- Battery monitoring on remote to prevent unexpected shutdowns

## Requirements
- ESP-NOW library
- WiFi library