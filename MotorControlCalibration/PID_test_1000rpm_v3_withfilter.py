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
    'pid_params': {
        'p': {'Kp': 52.64, 'alpha': 0.3},  
        'pi': {'Kp': 30.00, 'Ki': 200.00, 'alpha': 0.4}, 
        'pid': {'Kp': 40.16, 'Ki': 374.92, 'Kd': 2.10, 'alpha': 0.2}  
    }
}

class PIDController:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, sample_time=0.02):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # Anti-windup limits
        self.output_limits = (-255, 255)
        self.integral_limits = (-10, 10)
        
        # Derivative filter parameters
        self.alpha = 0.3  # Filter coefficient (0 = no filtering, 1 = max filtering)
        self.filtered_derivative = 0.0
        
    def reset(self):
        """Reset controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        
    def compute(self, setpoint, measured_value, current_time=None):
        """Compute PID control output with filtered derivative."""
        if current_time is None:
            current_time = time.time()
            
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = setpoint - measured_value
            return 0
            
        # Time delta
        dt = current_time - self.last_time
        if dt < self.sample_time:
            return None
            
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral = np.clip(self.integral + error * dt, 
                              self.integral_limits[0], 
                              self.integral_limits[1])
        I = self.Ki * self.integral
        
        # Derivative term with low-pass filter
        if dt > 0:
            current_derivative = (error - self.last_error) / dt
            # Apply low-pass filter
            self.filtered_derivative = (self.alpha * current_derivative + 
                                     (1 - self.alpha) * self.filtered_derivative)
        else:
            self.filtered_derivative = 0
            
        D = self.Kd * self.filtered_derivative
        
        # Calculate total output
        output = P + I + D
        
        # Clip output to limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self._print_config()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print("Connected to serial port")
        
        self.last_position = None
        self.last_time = None
        
    def _print_config(self):
        print("\n=== Motor Configuration ===")
        print(f"Motor: {CONFIG['motor_model']} ({CONFIG['motor_sku']})")
        print(f"Rated: {CONFIG['rated_rpm']} RPM @ {CONFIG['rated_voltage']}V")
        print(f"Target velocity: {CONFIG['setpoint_velocity']} m/s")
        
    def send_motor_command(self, pwm):
        """Send PWM command to motor with right-side inversion"""
        pwm = int(-pwm)  # Negate PWM for right motor
        cmd = f"M,{pwm}\n"
        self.ser.write(cmd.encode())
        
    def read_encoder(self):
        """Read encoder value with error handling"""
        try:
            line = self.ser.readline().decode().strip()
            if line.startswith("E,"):
                return int(line.split(",")[1])
            return None
        except:
            return None
            
    def get_velocity(self, current_position, current_time=None):
        """Calculate velocity from encoder counts"""
        if current_time is None:
            current_time = time.time()
            
        if self.last_position is None:
            self.last_position = current_position
            self.last_time = current_time
            return 0.0
            
        dt = current_time - self.last_time
        if dt >= CONFIG['sample_time']:
            counts_per_sec = (current_position - self.last_position) / dt
            rps = counts_per_sec / CONFIG['encoder_cpr']
            velocity = rps * CONFIG['wheel_circumference']
            
            self.last_position = current_position
            self.last_time = current_time
            
            return velocity
            
        return None

def run_pid_test(pid_type='pid'):
    """Run PID control test with specified parameters"""
    params = CONFIG['pid_params'][pid_type]
    controller = PIDController(
        Kp=params.get('Kp', 0),
        Ki=params.get('Ki', 0),
        Kd=params.get('Kd', 0),
        sample_time=CONFIG['sample_time']
    )
    controller.alpha = params.get('alpha', 0.3)  # Set alpha from config
    
    motor = MotorController()
    
    times = []
    velocities = []
    setpoints = []
    pwm_values = []
    
    start_time = time.time()
    print(f"\nStarting {pid_type.upper()} control test...")
    
    try:
        controller.reset()  # Ensure fresh start
        while (time.time() - start_time) < CONFIG['test_duration']:
            current_time = time.time()
            position = motor.read_encoder()
            
            if position is not None:
                velocity = motor.get_velocity(position, current_time)
                
                if velocity is not None:
                    # Compute PID output (PWM value)
                    pwm = controller.compute(CONFIG['setpoint_velocity'], 
                                          velocity, 
                                          current_time)
                    
                    if pwm is not None:
                        # Send command to motor
                        motor.send_motor_command(pwm)
                        
                        # Store data
                        times.append(current_time - start_time)
                        velocities.append(velocity)
                        setpoints.append(CONFIG['setpoint_velocity'])
                        pwm_values.append(pwm)
                        
    except KeyboardInterrupt:
        print("\nTest interrupted!")
    finally:
        motor.send_motor_command(0)
        motor.ser.close()
        
    return np.array(times), np.array(velocities), np.array(setpoints), np.array(pwm_values)

def plot_results(results, save_dir):
    """Plot results with comprehensive parameter information."""
    # Define consistent colors for each controller type
    COLORS = {
        'p': 'blue',
        'pi': 'green',
        'pid': 'purple'
    }
    
    plt.figure(figsize=(15, 12))
    
    # Create main title with all parameters
    main_title = f"PID Step Response Test V3 - 1000RPM)\n"
    main_title += f"Motor: {CONFIG['motor_model']} ({CONFIG['motor_sku']})\n"
    main_title += f"{CONFIG['rated_rpm']} RPM @ {CONFIG['rated_voltage']}V, "
    main_title += f"{CONFIG['gear_ratio']}:1 ratio, "
    main_title += f"{CONFIG['wheel_diameter']*1000:.0f}mm wheel\n"
    main_title += f"Test Voltage: {CONFIG['test_voltage']}V, "  # Added comma
    main_title += f"Sample Time: {CONFIG['sample_time']*1000:.0f}ms\n"  # Added sample time
    
    # Add controller parameters
    param_text = "Controller Parameters:\n"
    for pid_type, params in CONFIG['pid_params'].items():
        param_text += f"{pid_type.upper()}: "
        param_list = []
        for k, v in params.items():
            if k != 'alpha':  # Handle alpha separately
                param_list.append(f"{k}={v:.2f}")
        param_list.append(f"α={params['alpha']:.2f}")  # Add alpha
        param_text += ", ".join(param_list)
        param_text += "\n"
    
    # Plot velocity responses
    plt.subplot(311)
    plt.title('Linear Velocity Response with PID Control\n' + main_title + param_text, pad=20)
    
    # First plot all setpoint lines
    for pid_type, data in results.items():
        plt.plot(data['times'], data['setpoints'], '--', 
                color='red', alpha=0.5, label='Setpoint' if pid_type == 'p' else '')
    
    # Then plot all velocity responses
    for pid_type, data in results.items():
        plt.plot(data['times'], data['velocities'], 
                color=COLORS[pid_type], 
                label=f'{pid_type.upper()} Control')
    
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.legend()
    
    # Plot PWM commands
    plt.subplot(312)
    plt.title('PWM Commands')
    for pid_type, data in results.items():
        plt.plot(data['times'], data['pwm_values'], 
                color=COLORS[pid_type], 
                label=f'{pid_type.upper()} PWM')
    
    plt.ylabel('PWM Command')
    plt.grid(True)
    plt.legend()
    
    # Plot errors
    plt.subplot(313)
    plt.title('Control Error')
    for pid_type, data in results.items():
        error = data['setpoints'] - data['velocities']
        plt.plot(data['times'], error,
                color=COLORS[pid_type],
                label=f'{pid_type.upper()} Error')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m/s)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(save_dir, f'pid_test_results_{timestamp}.png')
    plt.savefig(filename, bbox_inches='tight', dpi=300)
    plt.show()

def save_results(results, save_dir):
    """Save test results to JSON and CSV files"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Calculate metrics for each test
    for pid_type, data in results.items():
        # Convert numpy arrays to lists for JSON serialization
        data['times'] = data['times'].tolist()
        data['velocities'] = data['velocities'].tolist()
        data['setpoints'] = data['setpoints'].tolist()
        data['pwm_values'] = data['pwm_values'].tolist()
        
        # Use last second for steady-state calculations
        steady_idx = np.array(data['times']) > (data['times'][-1] - 1.0)
        steady_velocity = np.mean(np.array(data['velocities'])[steady_idx])
        
        # Calculate error metrics
        error = np.array(data['velocities']) - np.array(data['setpoints'])
        mse = np.mean(error**2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(error))
        
        data['metrics'] = {
            'steady_velocity': float(steady_velocity),
            'mse': float(mse),
            'rmse': float(rmse),
            'mae': float(mae),
            'alpha': CONFIG['pid_params'][pid_type]['alpha']  # Add alpha to metrics
        }
    
    # Save full results to JSON
    json_data = {
        'config': CONFIG,
        'results': results,
        'timestamp': timestamp
    }
    
    json_file = os.path.join(save_dir, f'pid_test_results_{timestamp}.json')
    with open(json_file, 'w') as f:
        json.dump(json_data, f, indent=4)
    
    # Save summary to CSV
    csv_file = os.path.join(save_dir, f'pid_test_summary_{timestamp}.csv')
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['PID Type', 'Steady Velocity (m/s)', 'RMSE', 'MAE', 'Alpha'])  # Added Alpha
        for pid_type, data in results.items():
            metrics = data['metrics']
            writer.writerow([
                pid_type,
                metrics['steady_velocity'],
                metrics['rmse'],
                metrics['mae'],
                metrics['alpha']  # Add alpha to CSV output
            ])
def main():
    results = {}
    
    try:
        # Run tests for each PID type
        for pid_type in CONFIG['pid_params'].keys():
            print(f"\nTesting {pid_type.upper()} control...")
            times, velocities, setpoints, pwm_values = run_pid_test(pid_type)
            
            results[pid_type] = {
                'times': times,
                'velocities': velocities,
                'setpoints': setpoints,
                'pwm_values': pwm_values
            }
            
            # Print immediate feedback
            if len(times) > 0:
                steady_idx = times > (times[-1] - 1.0)
                steady_velocity = np.mean(velocities[steady_idx])
                print(f"Steady-state velocity: {steady_velocity:.3f} m/s")
        
        # Plot and save results
        plot_results(results, save_dir)
        save_results(results, save_dir)
        
    except KeyboardInterrupt:
        print("\nTesting interrupted!")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("\nTesting complete")

if __name__ == "__main__":
    main()