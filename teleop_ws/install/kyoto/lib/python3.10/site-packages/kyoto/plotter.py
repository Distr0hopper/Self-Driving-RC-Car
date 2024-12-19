#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import json
import tkinter as tk
from tkinter import ttk
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time

class PIDObserver(Node):
    def __init__(self):
        super().__init__('pid_observer')
        
        # Save directory
        self.save_dir = "/home/seb/Documents/Telerobotics/KyotoPIDTest"
        os.makedirs(self.save_dir, exist_ok=True)
        
        # QoS profile
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Constants
        self.WHEEL_DIAMETER = 0.127
        self.WHEEL_CIRCUMFERENCE = np.pi * self.WHEEL_DIAMETER
        self.ENCODER_CPR = 204.19
        
        # Current state values (updated by callbacks)
        self.current_setpoint = 0.0
        self.current_pwm = 0.0
        self.current_velocity = 0.0
        
        # Data storage
        self.reset_data()
        
        # Subscribe to topics
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'motor_pwm', self.pwm_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, self.qos)
        
        # Create 50Hz timer for data recording
        self.create_timer(0.02, self.record_data)  # 50Hz
        
        # Create GUI
        self.create_gui()
        
    def reset_data(self):
        """Reset all data arrays"""
        self.times = []
        self.setpoints = []
        self.velocities = []
        self.pwm_values = []
        
        # For velocity calculation
        self.last_encoder_counts = None
        self.last_encoder_time = None
        
        # Recording state
        self.is_recording = False
        self.start_time = None
        
    def vel_callback(self, msg):
        """Update current setpoint"""
        self.current_setpoint = msg.data
        
    def pwm_callback(self, msg):
        """Update current PWM"""
        self.current_pwm = msg.data[0]  # Front right motor
        
    def get_velocity(self, counts):
        """Calculate velocity from encoder counts"""
        current_time = time.time()
        
        if self.last_encoder_counts is None:
            self.last_encoder_counts = counts
            self.last_encoder_time = current_time
            return 0.0
            
        dt = current_time - self.last_encoder_time
        if dt >= 0.02:  # 50Hz minimum
            count_diff = counts - self.last_encoder_counts
            counts_per_sec = count_diff / dt
            rps = counts_per_sec / self.ENCODER_CPR
            velocity = rps * self.WHEEL_CIRCUMFERENCE
            
            self.last_encoder_counts = counts
            self.last_encoder_time = current_time
            
            return velocity
        return None
        
    def encoder_callback(self, msg):
        """Update current velocity"""
        velocity = self.get_velocity(msg.data[0])
        if velocity is not None:
            self.current_velocity = velocity
            
    def record_data(self):
        """Record data at 50Hz"""
        if self.is_recording and self.start_time is not None:
            current_time = time.time() - self.start_time
            self.times.append(current_time)
            self.setpoints.append(self.current_setpoint)
            self.velocities.append(self.current_velocity)
            self.pwm_values.append(self.current_pwm)
            
    def create_gui(self):
        """Create Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("PID Response Observer")
        
        # Control frame
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.record_button = ttk.Button(control_frame, text="Start Recording", 
                                      command=self.toggle_recording)
        self.record_button.grid(row=0, column=0, padx=5)
        
        # Create figure for plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0)
        
        # Update GUI periodically (10Hz is enough for display)
        self.root.after(100, self.update_plot)
        
    def toggle_recording(self):
        """Start or stop recording"""
        self.is_recording = not self.is_recording
        
        if self.is_recording:
            self.reset_data()
            self.start_time = time.time()
            self.record_button.config(text="Stop Recording")
        else:
            self.record_button.config(text="Start Recording")
            self.save_data()
            
    def update_plot(self):
        """Update plot with current data"""
        if len(self.times) > 0:
            # Clear previous plots
            self.ax1.clear()
            self.ax2.clear()
            
            # Main title
            self.fig.suptitle('Real-time PID Response', fontsize=14)
            
            # Plot velocity
            self.ax1.plot(self.times, self.setpoints, 'r--', label='Setpoint')
            self.ax1.plot(self.times, self.velocities, 'b-', label='Measured')
            self.ax1.set_title('Velocity Tracking')
            self.ax1.set_ylabel('Velocity (m/s)')
            self.ax1.grid(True)
            self.ax1.legend()
            
            # Plot PWM
            self.ax2.plot(self.times, self.pwm_values, 'g-', label='PWM')
            self.ax2.axhline(y=255, color='r', linestyle=':', label='PWM Limits')
            self.ax2.axhline(y=-255, color='r', linestyle=':')
            self.ax2.set_title('PWM Control Signal')
            self.ax2.set_ylabel('PWM Value')
            self.ax2.set_xlabel('Time (s)')
            self.ax2.grid(True)
            self.ax2.legend()
            
            # Adjust layout
            self.fig.tight_layout()
            self.canvas.draw()
            
        # Schedule next update
        self.root.after(100, self.update_plot)
        
    def save_data(self):
        """Save recorded data"""
        if len(self.times) > 0:
            # Create timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Calculate duration and stats
            duration = self.times[-1]
            max_vel = max(abs(min(self.velocities)), abs(max(self.velocities)))
            max_pwm = max(abs(min(self.pwm_values)), abs(max(self.pwm_values)))
            
            # Create descriptive filename
            base_name = f"pid_test_{duration:.1f}s_{max_vel:.1f}ms"
            
            # Save plot with detailed information
            plt.figure(figsize=(12, 8))
            
            # Main title with test information
            plt.suptitle('PID Response Test Results\n' + 
                        f'Duration: {duration:.1f}s, Max Velocity: {max_vel:.1f} m/s, Max PWM: {max_pwm:.0f}',
                        fontsize=14)
            
            # Velocity subplot
            plt.subplot(211)
            plt.plot(self.times, self.setpoints, 'r--', label='Setpoint')
            plt.plot(self.times, self.velocities, 'b-', label='Measured')
            plt.title('Velocity Tracking')
            plt.ylabel('Velocity (m/s)')
            plt.legend()
            plt.grid(True)
            
            # PWM subplot
            plt.subplot(212)
            plt.plot(self.times, self.pwm_values, 'g-', label='PWM')
            plt.axhline(y=255, color='r', linestyle=':', label='PWM Limits')
            plt.axhline(y=-255, color='r', linestyle=':')
            plt.title('PWM Control Signal')
            plt.ylabel('PWM Value')
            plt.xlabel('Time (s)')
            plt.legend()
            plt.grid(True)
            
            # Add timestamp and file info
            plt.figtext(0.99, 0.01, f'Generated: {timestamp}', 
                       ha='right', va='bottom', fontsize=8)
            
            # Adjust layout and save
            plt.tight_layout()
            plot_file = os.path.join(self.save_dir, f"{base_name}_{timestamp}.png")
            plt.savefig(plot_file, bbox_inches='tight', dpi=300)
            plt.close()
            
            # Save data to JSON
            data = {
                'timestamp': timestamp,
                'duration': duration,
                'max_velocity': max_vel,
                'max_pwm': max_pwm,
                'sample_rate': 50,  # Hz
                'data': {
                    'times': self.times,
                    'setpoints': self.setpoints,
                    'velocities': self.velocities,
                    'pwm_values': self.pwm_values
                }
            }
            
            json_file = os.path.join(self.save_dir, f"{base_name}_{timestamp}.json")
            with open(json_file, 'w') as f:
                json.dump(data, f, indent=4)
                
            self.get_logger().info(
                f'Data saved:\n'
                f'Plot: {plot_file}\n'
                f'Data: {json_file}\n'
                f'Duration: {duration:.1f}s, Max Velocity: {max_vel:.1f} m/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PIDObserver()
    
    # Run ROS2 in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Run GUI in main thread
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()