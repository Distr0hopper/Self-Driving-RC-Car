#!/usr/bin/env python3
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os
from datetime import datetime


# Create directory for saving files
save_dir = "/home/seb/Documents/Telerobotics/stepper_test_ws/pid_test_results"
os.makedirs(save_dir, exist_ok=True)

def analyze_step_response(times, velocities, pwm):
    """Analyze step response to find L, T and K parameters."""
    
    # Find steady state value (average of last second)
    steady_idx = times > (times[-1] - 1.0)
    steady_velocity = np.mean(velocities[steady_idx])
    
    # Calculate K (process gain)
    K = steady_velocity / pwm  # m/s per PWM
    
    # Find time to reach 63.2% of final value (time constant T)
    target_velocity = 0.632 * steady_velocity
    idx = np.argmin(np.abs(velocities - target_velocity))
    T = times[idx]
    
    # Find dead time (L) - time to reach 5% of final value
    threshold = 0.05 * steady_velocity
    for i, v in enumerate(velocities):
        if v > threshold:
            L = times[i]
            break
    
    return K, L, T, steady_velocity

def plot_step_response_with_params(times, velocities, K, L, T, pwm, pid_params, steady_velocity):
    """Plot step response with annotations and PID parameters."""
    fig = plt.figure(figsize=(15, 10))
    
    # Create main plot
    ax1 = plt.subplot2grid((3, 3), (0, 0), colspan=3, rowspan=2)
    
    # Plot velocity data
    ax1.plot(times, velocities, 'b-', label='Velocity', linewidth=2)
    
    # Plot 63.2% line
    ax1.axhline(y=0.632*steady_velocity, color='r', linestyle='--', 
                label='63.2% of steady state')
    
    # Plot 5% line for dead time
    ax1.axhline(y=0.05*steady_velocity, color='g', linestyle=':', 
                label='5% of steady state')
    
    # Plot T and L
    ax1.plot([L, L], [0, steady_velocity*0.05], 'g-', linewidth=2, label='L (dead time)')
    ax1.plot([T, T], [0, 0.632*steady_velocity], 'r-', linewidth=2, label='T (time constant)')
    
    # Formatting
    ax1.set_title(f'DC Motor Step Response Analysis (PWM={pwm})', fontsize=14, pad=20)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.grid(True)
    ax1.legend(loc='lower right')
    
    # Add text box with parameters
    params_text = f"System Parameters:\n"
    params_text += f"K (process gain) = {K:.4f} (m/s)/PWM\n"
    params_text += f"L (dead time) = {L:.4f} s\n"
    params_text += f"T (time constant) = {T:.4f} s\n"
    params_text += f"Steady State Velocity = {steady_velocity:.4f} m/s"
    
    # Add parameters text box
    plt.figtext(0.1, 0.25, params_text, fontsize=12, 
                bbox=dict(facecolor='white', alpha=0.8))
    
    # Add PID parameters
    pid_text = "Ziegler-Nichols PID Parameters:\n\n"
    for method, params in pid_params.items():
        pid_text += f"{method.upper()} Control:\n"
        for param, value in params.items():
            pid_text += f"{param} = {value:.4f}\n"
        pid_text += "\n"
    
    plt.figtext(0.65, 0.25, pid_text, fontsize=12,
                bbox=dict(facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    
    # Save the plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(os.path.join(save_dir, f'motor_analysis_{timestamp}.png'), 
                bbox_inches='tight', dpi=300)
    plt.show()

def calculate_pid_parameters(K, L, T):
    """Calculate PID parameters using Ziegler-Nichols method."""
    
    # Classic Ziegler-Nichols
    zn_params = {
        'P': {'Kp': T/(K*L)},
        'PI': {'Kp': 0.9*T/(K*L), 'Ti': L/0.3},
        'PID': {'Kp': 1.2*T/(K*L), 'Ti': 2*L, 'Td': 0.5*L}
    }
    
    # Calculate Ki and Kd from Ti and Td
    for method in ['PI', 'PID']:
        if method in zn_params:
            params = zn_params[method]
            if 'Ti' in params:
                params['Ki'] = params['Kp'] / params['Ti']
            if 'Td' in params:
                params['Kd'] = params['Kp'] * params['Td']
    
    return zn_params

def main():
    # Load JSON data from step response test
    json_path = "/home/seb/Documents/Telerobotics/stepper_test_ws/step_response_results/jga25_371_12v0_20241208_044220.json"
    
    print(f"Loading data from: {json_path}")
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Look for PWM=255 test data
    test_data = None
    for test in data['test_data']:
        if test['pwm'] == 255:
            test_data = test
            break
    
    if test_data is None:
        print("Could not find PWM=255 test data")
        return
    
    times = np.array(test_data['data']['times'])
    velocities = np.array(test_data['data']['velocity_mps'])
    pwm = test_data['pwm']
    
    # Analyze step response
    K, L, T, steady_velocity = analyze_step_response(times, velocities, pwm)
    
    # Calculate PID parameters
    pid_params = calculate_pid_parameters(K, L, T)
    
    # Plot analysis with parameters
    plot_step_response_with_params(times, velocities, K, L, T, pwm, 
                                 pid_params, steady_velocity)
    
    # Print results to console
    print("\nStep Response Analysis:")
    print(f"K (process gain) = {K:.4f} (m/s)/PWM")
    print(f"L (dead time) = {L:.4f} s")
    print(f"T (time constant) = {T:.4f} s")
    print(f"Steady State Velocity = {steady_velocity:.4f} m/s")
    
    print("\nZiegler-Nichols PID Parameters:")
    for method, params in pid_params.items():
        print(f"\n{method} Control:")
        for param, value in params.items():
            print(f"{param} = {value:.4f}")
            
    # Save PID parameters to JSON
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pid_params_file = os.path.join(save_dir, f'pid_parameters_{timestamp}.json')
    with open(pid_params_file, 'w') as f:
        json.dump({
            'system_parameters': {
                'K': float(K),
                'L': float(L),
                'T': float(T),
                'steady_velocity': float(steady_velocity)
            },
            'pid_parameters': pid_params
        }, f, indent=4)
    
    print(f"\nPID parameters saved to: {pid_params_file}")

if __name__ == "__main__":
    main()