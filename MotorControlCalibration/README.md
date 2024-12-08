# Motor Control Calibration

This guide explains how to calibrate and tune PID control for DC motors with encoders. The process is broken down into 4 steps.

## Step 1: Encoder Calibration
First, we need to calibrate the encoder to determine the ticks per wheel revolution and gear ratio.

- **Hardware**: ESP32 + Hall Effect sensor + DC Motor with encoder
- **Code**: `encoder_calibration/encoder_quadrature_test_hall/encoder_quadrature_test_hall.ino`
- **Process**: 
  - Hall sensor detects each motor revolution
  - Code counts encoder ticks per revolution
  - Automatically calculates gear ratio
  - Results show pulses/rev and final gear ratio

## Step 2: Step Response Test
Next, we perform a step response test to understand how the motor responds to input.

- **Files**:
  - Python: `step_response_1000rpm.py`
  - ESP32: `Velocity_PID_test/src/main.cpp`
- **Process**:
  - Sends maximum PWM command to motor
  - Records velocity response over time
  - Generates velocity vs time plots
  - Saves raw data for PID tuning

## Step 3: PID Parameter Calculation
Using the step response data, we calculate initial PID values using Ziegler-Nichols method.

- **Code**: `pid_tuner_informative.py`
- **What it does**:
  - Analyzes step response data
  - Calculates dead time (L) and time constant (T)
  - Generates suggested P, PI, and PID parameters
  - Creates detailed analysis plots
- **Output**: Initial PID values for testing

## Step 4: PID Testing and Fine-Tuning
Finally, we test and fine-tune the calculated PID values on the actual motor.

- **Files**:
  - Python: `PID_test_1000rpm_v4.py`
  - ESP32: `Velocity_PID_test/src/main.cpp`
- **Process**:
  - Tests P, PI, and PID control
  - Records and plots performance
  - Allows parameter adjustment
  - Saves test results for comparison

Results for each step are saved in their respective folders:
- Step 1: Calibration results printed to serial
- Step 2: `/step_response_results/`
- Step 3: `/pid_tune_results/`
- Step 4: `/pid_test_results/`

## Usage
1. Run encoder calibration first to get accurate motor parameters
2. Perform step response test and save data
3. Use tuner to calculate initial PID values
4. Test and adjust PID values until desired performance is achieved

Each script includes clear instructions and parameter settings at the top of the file.