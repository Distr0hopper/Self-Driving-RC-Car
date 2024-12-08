#!/usr/bin/env python3
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import json
import csv
from datetime import datetime
import os

# Create directory for saving files
save_dir = "/home/seb/Documents/Telerobotics/stepper_test_ws/step_response_results"
os.makedirs(save_dir, exist_ok=True)

CONFIG = {
    'motor_model': 'JGA25-371',
    'motor_sku': 'gs20179-01',
    'rated_voltage': 12.0,
    'rated_rpm': 1000,
    'test_voltage': 12.0,
    'gear_ratio': 4.64,
    'output_rpm': 1000/4.64,
    'encoder_ppr': 11,
    'encoder_cpr': 204.19,
    'wiring': {
        'encoder_a': 'Yellow',
        'encoder_b': 'Green'
    },
    'wheel_diameter': 0.244,
    'wheel_circumference': np.pi * 0.244,
    'test_duration': 3.0,
    'sample_time': 0.02,
    #'pwm_range': list(range(50, 256, 50))  # Now includes 255 for max PWM
    'pwm_range': [255]  # Single test at maximum PWM
}

class MotorTuner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        print("\n=== Motor Configuration ===")
        print(f"Motor: {CONFIG['motor_model']} ({CONFIG['motor_sku']})")
        print(f"Rated: {CONFIG['rated_rpm']} RPM @ {CONFIG['rated_voltage']}V")
        print(f"Gear Ratio: {CONFIG['gear_ratio']}:1")
        print(f"Output: {CONFIG['output_rpm']:.1f} RPM")
        print(f"Test Voltage: {CONFIG['test_voltage']}V")
        print("\n=== Encoder ===")
        print(f"PPR: {CONFIG['encoder_ppr']}")
        print(f"CPR: {CONFIG['encoder_cpr']} (calibrated)")
        print(f"Wiring: A→{CONFIG['wiring']['encoder_a']}, B→{CONFIG['wiring']['encoder_b']}")
        print("\n=== Mechanical ===")
        print(f"Wheel: {CONFIG['wheel_diameter']*1000:.0f}mm diameter")
        print(f"     : {CONFIG['wheel_circumference']*1000:.0f}mm circumference")
        
        print(f"\nConnecting to {port} at {baudrate} baud...")
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print("Connected to serial port")

    def send_pwm(self, pwm):
        cmd = f"M,{int(-pwm)}\n"
        print(f"Sending command: {cmd.strip()}")
        self.ser.write(cmd.encode())
        
    def read_encoder(self):
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("E,"):
                return int(line.split(",")[1])
            return None
        except Exception as e:
            print(f"Error reading serial: {e}")
            return None
        
    def counts_to_rps(self, counts_per_sec):
        return counts_per_sec / CONFIG['encoder_cpr']
        
    def counts_to_velocity(self, counts_per_sec):
        rps = self.counts_to_rps(counts_per_sec)
        return rps * CONFIG['wheel_circumference']
        
    def step_test(self, pwm):
        print(f"\nStarting step test with PWM = {pwm}")
        
        times = []
        positions = []
        velocities = []
        rps_values = []
        reading_count = 0  # Track number of readings to handle initialization
        
        print("Ensuring motor is stopped...")
        self.send_pwm(0)
        time.sleep(0.5)
        
        # Get initial position BEFORE sending PWM command
        initial_pos = self.read_encoder()
        if initial_pos is None:
            print("Failed to get initial position")
            return [], [], [], []
            
        # Initialize with zero velocity
        start_time = time.time()
        last_pos = initial_pos
        last_time = start_time
        
        # Add zero velocity point
        times.append(0)
        positions.append(initial_pos)
        velocities.append(0)
        rps_values.append(0)
        
        print(f"Starting test: PWM = {pwm}")
        self.send_pwm(pwm)
        
        # Skip first few readings to let velocity calculation stabilize
        readings_to_skip = 2 if pwm >= 150 else 1
        
        while (time.time() - start_time) < CONFIG['test_duration']:
            current_time = time.time()
            pos = self.read_encoder()
            
            if pos is not None:
                dt = current_time - last_time
                if dt >= CONFIG['sample_time']:
                    reading_count += 1
                    
                    if reading_count > readings_to_skip:
                        counts_per_sec = (pos - last_pos) / dt
                        rps = self.counts_to_rps(counts_per_sec)
                        velocity = rps * CONFIG['wheel_circumference']
                        
                        times.append(current_time - start_time)
                        positions.append(pos)
                        velocities.append(velocity)
                        rps_values.append(rps)
                    
                    last_pos = pos
                    last_time = current_time
        
        print("Test complete, stopping motor")
        self.send_pwm(0)
        time.sleep(0.5)
        
        return np.array(times), np.array(positions), np.array(velocities), np.array(rps_values)

    def run_tests(self):
        results = []
        
        plt.figure(figsize=(15, 10))
        
        main_title = f"Motor: {CONFIG['motor_model']} ({CONFIG['motor_sku']})\n"
        main_title += f"{CONFIG['rated_rpm']} RPM @ {CONFIG['rated_voltage']}V, "
        main_title += f"{CONFIG['gear_ratio']}:1 ratio, "
        main_title += f"{CONFIG['wheel_diameter']*1000:.0f}mm wheel\n"
        main_title += f"Test Voltage: {CONFIG['test_voltage']}V"
        plt.suptitle(main_title, y=0.98)
        
        plt.subplot(211)
        plt.title('Linear Velocity Response')
        
        plt.subplot(212)
        plt.title('Angular Velocity Response')
        
        for pwm in CONFIG['pwm_range']:
            times, positions, velocities, rps = self.step_test(pwm)
            
            if len(times) > 0:
                steady_idx = times > (times[-1] - 1.0)
                steady_velocity = np.mean(velocities[steady_idx])
                steady_rps = np.mean(rps[steady_idx])
                steady_rpm = steady_rps * 60
                
                results.append({
                    'pwm': pwm,
                    'steady_values': {
                        'velocity_mps': float(steady_velocity),
                        'rps': float(steady_rps),
                        'rpm': float(steady_rpm)
                    },
                    'data': {
                        'times': times.tolist(),
                        'velocity_mps': velocities.tolist(),
                        'rps': rps.tolist()
                    }
                })
                
                plt.subplot(211)
                plt.plot(times, velocities, label=f'PWM={pwm}')
                
                plt.subplot(212)
                plt.plot(times, rps, label=f'PWM={pwm}')
                
                print(f"\nPWM={pwm}:")
                print(f"Steady-state values:")
                print(f"  Velocity: {steady_velocity:.2f} m/s")
                print(f"  Speed: {steady_rpm:.1f} RPM ({steady_rps:.2f} RPS)")
        
        plt.subplot(211)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.grid(True)
        plt.legend()
        
        plt.subplot(212)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (RPS)')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        model_name = CONFIG['motor_model'].replace('-', '_').lower()
        voltage = str(CONFIG['test_voltage']).replace('.', 'v')
        filename_base = os.path.join(save_dir, f'{model_name}_{voltage}_{timestamp}')
        
        plt.savefig(f'{filename_base}.png', bbox_inches='tight', dpi=300)
        plt.show()
        
        full_results = {
            'config': CONFIG,
            'test_data': results,
            'timestamp': timestamp
        }
        
        with open(f'{filename_base}.json', 'w') as f:
            json.dump(full_results, f, indent=4)
            
        with open(f'{filename_base}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['PWM', 'Velocity (m/s)', 'Speed (RPS)', 'Speed (RPM)'])
            for result in results:
                writer.writerow([
                    result['pwm'],
                    result['steady_values']['velocity_mps'],
                    result['steady_values']['rps'],
                    result['steady_values']['rpm']
                ])
                
        return results

def main():
    try:
        print("\nStarting Motor Characterization...")
        tuner = MotorTuner()
        results = tuner.run_tests()
        
    except KeyboardInterrupt:
        print("\nTest interrupted!")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            tuner.send_pwm(0)
            tuner.ser.close()
            print("Serial port closed")
        except:
            pass

if __name__ == "__main__":
    main()