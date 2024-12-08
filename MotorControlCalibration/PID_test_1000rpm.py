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
save_dir = "/home/seb/Documents/Telerobotics/stepper_test_ws/pid_test_results"
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
    'setpoint_velocity': 3.0,  # m/s target velocity
    'motor_pwm': 255,  # PWM value to use when running motor
    # Ziegler-Nichols values from tuning
    'pid_params': {
        'p': {'Kp': 52.64},  
        'pi': {'Kp': 48.00, 'Ki': 214.00}, 
        'pid': {'Kp': 63.16, 'Ki': 474.92, 'Kd': 2.10}  
    }
}

class PIDTuner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self._print_config()
        self._setup_serial(port, baudrate)

    def _print_config(self):
        """Print configuration details."""
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

    def _setup_serial(self, port, baudrate):
        """Setup serial connection with debug information."""
        print(f"\nConnecting to {port} at {baudrate} baud...")
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected to serial port")
        
        # Flush any pending data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send_command(self, cmd_type, value):
        """Send command to microcontroller with debug information."""
        cmd = f"{cmd_type},{value}\n"
        print(f"DEBUG: Sending command: {cmd.strip()}")
        self.ser.write(cmd.encode())
        time.sleep(0.1)  # Small delay to ensure command is processed
        
        # Read and print any response
        response = self.ser.readline().decode().strip()
        if response:
            print(f"DEBUG: Received response: {response}")
        
    def read_encoder(self):
        """Read encoder value with improved error handling."""
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("E,"):
                return int(line.split(",")[1])
            elif line:  # Print non-empty responses that aren't encoder values
                print(f"DEBUG: Unexpected response: {line}")
            return None
        except Exception as e:
            print(f"Error reading serial: {e}")
            return None
        
    def counts_to_rps(self, counts_per_sec):
        """Convert encoder counts per second to revolutions per second."""
        return counts_per_sec / CONFIG['encoder_cpr']
        
    def counts_to_velocity(self, counts_per_sec):
        """Convert encoder counts per second to linear velocity."""
        rps = self.counts_to_rps(counts_per_sec)
        return rps * CONFIG['wheel_circumference']
        
    def test_pid(self, pid_type, params):
        """Test a specific PID configuration."""
        print(f"\nStarting {pid_type} test with parameters:")
        for key, value in params.items():
            print(f"{key} = {value}")
        
        times = []
        positions = []
        velocities = []
        rps_values = []
        setpoints = []
        reading_count = 0
        
        # Reset any previous state
        print("Stopping motor...")
        self.send_command('M', 0)
        time.sleep(0.5)
        
        # Get initial position
        print("Getting initial position...")
        initial_pos = self.read_encoder()
        if initial_pos is None:
            print("Failed to get initial position")
            return [], [], [], [], []
            
        # Initialize with zero velocity
        start_time = time.time()
        last_pos = initial_pos
        last_time = start_time
        
        # Add zero velocity point
        times.append(0)
        positions.append(initial_pos)
        velocities.append(0)
        rps_values.append(0)
        setpoints.append(CONFIG['setpoint_velocity'])
        
        # Configure PID parameters
        print("\nConfiguring PID parameters...")
        for key, value in params.items():
            self.send_command(key.upper(), value)
            time.sleep(0.1)  # Give controller time to process each parameter
        
        # Send setpoint
        print(f"Setting velocity setpoint to {CONFIG['setpoint_velocity']} m/s")
        self.send_command('S', CONFIG['setpoint_velocity'])
        time.sleep(0.1)
        
        # Start motor with specified PWM
        print(f"Starting motor with PWM = {CONFIG['motor_pwm']}")
        self.send_command('M', -CONFIG['motor_pwm'])  # Added back negative sign for right-side motor
        
        print("Beginning data collection...")
        while (time.time() - start_time) < CONFIG['test_duration']:
            current_time = time.time()
            pos = self.read_encoder()
            
            if pos is not None:
                dt = current_time - last_time
                if dt >= CONFIG['sample_time']:
                    reading_count += 1
                    
                    counts_per_sec = (pos - last_pos) / dt
                    rps = self.counts_to_rps(counts_per_sec)
                    velocity = rps * CONFIG['wheel_circumference']
                    
                    times.append(current_time - start_time)
                    positions.append(pos)
                    velocities.append(velocity)
                    rps_values.append(rps)
                    setpoints.append(CONFIG['setpoint_velocity'])
                    
                    last_pos = pos
                    last_time = current_time
        
        print("Test complete, stopping motor")
        self.send_command('M', 0)
        time.sleep(0.5)
        
        return (np.array(times), np.array(positions), 
                np.array(velocities), np.array(rps_values), 
                np.array(setpoints))

    def run_tests(self):
        """Run all PID tests and generate plots and data files."""
        results = []
        
        plt.figure(figsize=(15, 10))
        
        main_title = f"Motor: {CONFIG['motor_model']} ({CONFIG['motor_sku']})\n"
        main_title += f"{CONFIG['rated_rpm']} RPM @ {CONFIG['rated_voltage']}V, "
        main_title += f"{CONFIG['gear_ratio']}:1 ratio, "
        main_title += f"{CONFIG['wheel_diameter']*1000:.0f}mm wheel\n"
        main_title += f"Test Voltage: {CONFIG['test_voltage']}V"
        plt.suptitle(main_title, y=0.98)
        
        plt.subplot(211)
        plt.title('Linear Velocity Response with PID Control')
        plt.subplot(212)
        plt.title('Angular Velocity Response with PID Control')
        
        for pid_type, params in CONFIG['pid_params'].items():
            (times, positions, velocities, 
             rps, setpoints) = self.test_pid(pid_type, params)
            
            if len(times) > 0:
                # Calculate performance metrics
                steady_idx = times > (times[-1] - 1.0)
                steady_velocity = np.mean(velocities[steady_idx])
                steady_rps = np.mean(rps[steady_idx])
                steady_rpm = steady_rps * 60
                
                # Calculate error metrics
                mse = np.mean((velocities - setpoints)**2)
                rmse = np.sqrt(mse)
                mae = np.mean(np.abs(velocities - setpoints))
                
                results.append({
                    'pid_type': pid_type,
                    'params': params,
                    'metrics': {
                        'steady_velocity_mps': float(steady_velocity),
                        'steady_rps': float(steady_rps),
                        'steady_rpm': float(steady_rpm),
                        'mse': float(mse),
                        'rmse': float(rmse),
                        'mae': float(mae)
                    },
                    'data': {
                        'times': times.tolist(),
                        'velocity_mps': velocities.tolist(),
                        'rps': rps.tolist(),
                        'setpoints': setpoints.tolist()
                    }
                })
                
                plt.subplot(211)
                plt.plot(times, velocities, label=f'{pid_type.upper()} Control')
                plt.plot(times, setpoints, '--', alpha=0.5)
                
                plt.subplot(212)
                plt.plot(times, rps, label=f'{pid_type.upper()} Control')
                
                print(f"\n{pid_type.upper()} Control Results:")
                print(f"Steady-state values:")
                print(f"  Velocity: {steady_velocity:.2f} m/s")
                print(f"  Speed: {steady_rpm:.1f} RPM ({steady_rps:.2f} RPS)")
                print(f"Error metrics:")
                print(f"  RMSE: {rmse:.4f} m/s")
                print(f"  MAE: {mae:.4f} m/s")
        
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
        
        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        model_name = CONFIG['motor_model'].replace('-', '_').lower()
        voltage = str(CONFIG['test_voltage']).replace('.', 'v')
        filename_base = os.path.join(save_dir, f'{model_name}_{voltage}_{timestamp}')
        
        # Save plot
        plt.savefig(f'{filename_base}.png', bbox_inches='tight', dpi=300)
        plt.show()
        
        # Save full results as JSON
        full_results = {
            'config': CONFIG,
            'test_data': results,
            'timestamp': timestamp
        }
        
        with open(f'{filename_base}.json', 'w') as f:
            json.dump(full_results, f, indent=4)
            
        # Save summary as CSV
        with open(f'{filename_base}.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['PID Type', 'Velocity (m/s)', 'Speed (RPS)', 
                           'Speed (RPM)', 'RMSE', 'MAE'])
            for result in results:
                writer.writerow([
                    result['pid_type'],
                    result['metrics']['steady_velocity_mps'],
                    result['metrics']['steady_rps'],
                    result['metrics']['steady_rpm'],
                    result['metrics']['rmse'],
                    result['metrics']['mae']
                ])
        
        return results

def main():
    """Main function with proper cleanup."""
    tuner = None
    try:
        print("\nStarting PID Testing...")
        tuner = PIDTuner()
        results = tuner.run_tests()
        
    except KeyboardInterrupt:
        print("\nTest interrupted!")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if tuner:
            try:
                tuner.send_command('M', 0)
                tuner.ser.close()
                print("Serial port closed")
            except:
                pass

if __name__ == "__main__":
    main()