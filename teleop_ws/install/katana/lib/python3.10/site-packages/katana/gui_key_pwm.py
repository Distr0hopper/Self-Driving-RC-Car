#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np

class GUIKeyPWM(Node):
    def __init__(self):
        super().__init__('gui_key_pwm')
        
        # Constants and limits
        self.MAX_PWM = 255
        self.MAX_STEERING = 100  # Steering PWM strength
        
        # QoS profile
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher
        self.pwm_pub = self.create_publisher(Int32MultiArray, 'motor_pwm', self.qos)
        
        # Control state
        self.forward = 0
        self.steering = 0
        self.ebrake = True  # Start with ebrake on
        
        # Create GUI
        self.create_gui()
        
        # Create control timer (50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(
            'PWM Control initialized\n'
            f'Max PWM: {self.MAX_PWM}\n'
            f'Steering strength: {self.MAX_STEERING}'
        )
        
    def create_gui(self):
        """Create Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("PWM Control")
        
        # Make window take focus for keyboard input
        self.root.focus_force()
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status indicators
        self.ebrake_label = ttk.Label(main_frame, text="E-BRAKE ON", foreground="red")
        self.ebrake_label.grid(row=0, column=0, columnspan=2, pady=5)
        
        self.forward_label = ttk.Label(main_frame, text="Forward PWM: 0")
        self.forward_label.grid(row=1, column=0, columnspan=2, pady=5)
        
        self.steering_label = ttk.Label(main_frame, text="Steering PWM: 0")
        self.steering_label.grid(row=2, column=0, columnspan=2, pady=5)
        
        # PWM adjustment sliders
        ttk.Label(main_frame, text="Max Forward PWM").grid(row=3, column=0, pady=5)
        self.max_pwm_slider = ttk.Scale(main_frame, from_=0, to=255, orient='horizontal')
        self.max_pwm_slider.set(self.MAX_PWM)
        self.max_pwm_slider.grid(row=3, column=1, pady=5, padx=5, sticky='ew')
        
        ttk.Label(main_frame, text="Steering Strength").grid(row=4, column=0, pady=5)
        self.steering_slider = ttk.Scale(main_frame, from_=0, to=200, orient='horizontal')
        self.steering_slider.set(self.MAX_STEERING)
        self.steering_slider.grid(row=4, column=1, pady=5, padx=5, sticky='ew')
        
        # Control instructions
        instructions = """
        Controls:
        W/S - Forward/Backward
        A/D - Left/Right
        Space - E-brake toggle
        
        Adjust sliders to change maximum values
        """
        ttk.Label(main_frame, text=instructions).grid(row=5, column=0, columnspan=2, pady=10)
        
        # Bind keyboard events
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        
    def key_press(self, event):
        """Handle key press events"""
        if event.keysym == 'w':
            self.forward = 1
        elif event.keysym == 's':
            self.forward = -1
        elif event.keysym == 'a':
            self.steering = 1
        elif event.keysym == 'd':
            self.steering = -1
        elif event.keysym == 'space':
            self.ebrake = not self.ebrake
            self.ebrake_label.config(
                text="E-BRAKE ON" if self.ebrake else "E-BRAKE OFF",
                foreground="red" if self.ebrake else "green"
            )
            
    def key_release(self, event):
        """Handle key release events"""
        if event.keysym in ['w', 's']:
            self.forward = 0
        elif event.keysym in ['a', 'd']:
            self.steering = 0
            
    def control_loop(self):
        """Generate and publish PWM commands"""
        # Get current max values from sliders
        self.MAX_PWM = int(self.max_pwm_slider.get())
        self.MAX_STEERING = int(self.steering_slider.get())
        
        # Update labels
        self.forward_label.config(text=f"Forward PWM: {self.forward*self.MAX_PWM}")
        self.steering_label.config(text=f"Steering PWM: {self.steering*self.MAX_STEERING}")
        
        if self.ebrake:
            # Send zero PWM to all motors
            pwm_msg = Int32MultiArray()
            pwm_msg.data = [0, 0, 0, 0]
            self.pwm_pub.publish(pwm_msg)
            return
            
        # Calculate base PWM from forward command
        base_pwm = int(self.forward * self.MAX_PWM)
        
        # Calculate steering PWM
        steer_pwm = int(self.steering * self.MAX_STEERING)
        
        # Calculate individual motor PWMs
        # Right side gets negative steering component
        fr_pwm = -int(np.clip(base_pwm + steer_pwm, -self.MAX_PWM, self.MAX_PWM))
        fl_pwm = int(np.clip(base_pwm - steer_pwm, -self.MAX_PWM, self.MAX_PWM))
        br_pwm = -int(np.clip(base_pwm + steer_pwm, -self.MAX_PWM, self.MAX_PWM))
        bl_pwm = int(np.clip(base_pwm - steer_pwm, -self.MAX_PWM, self.MAX_PWM))
        
        # Create message and publish
        pwm_msg = Int32MultiArray()
        pwm_msg.data = [fr_pwm, fl_pwm, br_pwm, bl_pwm]
        self.pwm_pub.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GUIKeyPWM()
    
    # Run ROS2 in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Run GUI in main thread
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        node.pwm_pub.publish(msg)
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()