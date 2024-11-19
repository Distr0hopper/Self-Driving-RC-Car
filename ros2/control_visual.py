#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray
from sensor_msgs.msg import Joy
import tkinter as tk
from tkinter import ttk
import math
import threading

class ControlVisualNode(Node):
    def __init__(self):
        super().__init__('control_visual')
        
        # Subscribe to controller topics
        self.speed_sub = self.create_subscription(
            Float32MultiArray,
            'current_speeds',
            self.speed_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            'controller_status',
            self.status_callback,
            10
        )
        
        self.threshold_sub = self.create_subscription(
            Float32MultiArray,
            'control_thresholds',
            self.threshold_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            10
        )
        
        # Initialize data
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.right_stick_y = 0.0  # Forward/back (swapped)
        self.left_stick_x = 0.0   # Rotation (swapped)
        self.turbo_active = False
        self.last_status = "Initializing..."
        self.speed_threshold = 1.0
        self.steering_threshold = 1.0
        
        # Encoder data
        self.encoder_counts = [0, 0, 0, 0]  # FR, FL, BR, BL
        self.ticks_per_rev = 990.0  # From bridge script
        
        # Create GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        self.get_logger().info('Control visualization node initialized')
        
    def speed_callback(self, msg):
        self.linear_speed = msg.data[0]
        self.angular_speed = msg.data[1]
        
    def joy_callback(self, msg):
        self.right_stick_y = msg.axes[4]  # Forward/back (swapped)
        self.left_stick_x = msg.axes[0]   # Rotation (swapped)
        self.turbo_active = bool(msg.buttons[5])  # RB button
        
    def status_callback(self, msg):
        self.last_status = msg.data
        
    def threshold_callback(self, msg):
        self.speed_threshold = msg.data[0]
        self.steering_threshold = msg.data[1]

    def encoder_callback(self, msg):
        self.encoder_counts = msg.data
        
    def create_gui(self):
        # Create main window
        self.root = tk.Tk()
        self.root.title("Katana UGV Control")
        self.root.geometry("1000x800")
        
        # Style
        style = ttk.Style()
        style.configure("TLabel", font=("Arial", 12))
        style.configure("Status.TLabel", font=("Arial", 14, "bold"))
        style.configure("Header.TLabel", font=("Arial", 12, "bold"))
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Status display
        self.status_label = ttk.Label(main_frame, text="Status: Initializing...", 
                                    style="Status.TLabel")
        self.status_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Canvas for stick visualization
        self.stick_canvas = tk.Canvas(main_frame, width=400, height=200, 
                                    bg='white', bd=2, relief='solid')
        self.stick_canvas.grid(row=1, column=0, columnspan=2, pady=10)
        
        # Speed displays
        speed_frame = ttk.Frame(main_frame)
        speed_frame.grid(row=2, column=0, columnspan=2, pady=10)
        
        ttk.Label(speed_frame, text="Linear Speed: ").grid(row=0, column=0)
        self.linear_speed_bar = ttk.Progressbar(speed_frame, length=200, mode='determinate')
        self.linear_speed_bar.grid(row=0, column=1)
        self.linear_speed_label = ttk.Label(speed_frame, text="0.0 m/s")
        self.linear_speed_label.grid(row=0, column=2)
        
        ttk.Label(speed_frame, text="Angular Speed: ").grid(row=1, column=0)
        self.angular_speed_bar = ttk.Progressbar(speed_frame, length=200, mode='determinate')
        self.angular_speed_bar.grid(row=1, column=1)
        self.angular_speed_label = ttk.Label(speed_frame, text="0.0 rad/s")
        self.angular_speed_label.grid(row=1, column=2)
        
        # Threshold displays
        threshold_frame = ttk.Frame(main_frame)
        threshold_frame.grid(row=3, column=0, columnspan=2, pady=10)
        
        ttk.Label(threshold_frame, text="Speed Threshold: ").grid(row=0, column=0)
        self.speed_threshold_bar = ttk.Progressbar(threshold_frame, length=200, mode='determinate')
        self.speed_threshold_bar.grid(row=0, column=1)
        self.speed_threshold_label = ttk.Label(threshold_frame, text="100%")
        self.speed_threshold_label.grid(row=0, column=2)
        
        ttk.Label(threshold_frame, text="Steering Threshold: ").grid(row=1, column=0)
        self.steering_threshold_bar = ttk.Progressbar(threshold_frame, length=200, mode='determinate')
        self.steering_threshold_bar.grid(row=1, column=1)
        self.steering_threshold_label = ttk.Label(threshold_frame, text="100%")
        self.steering_threshold_label.grid(row=1, column=2)

        # Encoder visualization
        encoder_frame = ttk.LabelFrame(main_frame, text="Wheel Encoders", padding="10")
        encoder_frame.grid(row=4, column=0, columnspan=2, pady=10, sticky="nsew")

        # Create a canvas for wheel rotation visualization
        self.wheel_canvas = tk.Canvas(encoder_frame, width=400, height=200, 
                                    bg='white', bd=2, relief='solid')
        self.wheel_canvas.grid(row=0, column=0, columnspan=4, pady=5)

        # Encoder counts and rotations
        self.encoder_labels = []
        self.rotation_labels = []
        wheel_names = ['FR', 'FL', 'BR', 'BL']
        
        for i, name in enumerate(wheel_names):
            ttk.Label(encoder_frame, text=f"{name}:").grid(row=1, column=i, padx=5)
            count_label = ttk.Label(encoder_frame, text="0")
            count_label.grid(row=2, column=i, padx=5)
            self.encoder_labels.append(count_label)
            
            rot_label = ttk.Label(encoder_frame, text="0.0 rot")
            rot_label.grid(row=3, column=i, padx=5)
            self.rotation_labels.append(rot_label)
        
        # Controls info
        controls_frame = ttk.Frame(main_frame)
        controls_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        controls_text = """
        Controls:
        - Right Stick: Forward/Backward
        - Left Stick: Rotation
        - D-Pad Up/Down: Adjust Speed Threshold
        - D-Pad Left/Right: Adjust Steering Threshold
        - RB: Turbo Mode
        - B: Emergency Stop
        """
        ttk.Label(controls_frame, text=controls_text).grid(row=0, column=0)
        
        # Turbo indicator
        self.turbo_label = ttk.Label(main_frame, text="TURBO", foreground='grey')
        self.turbo_label.grid(row=6, column=0, columnspan=2, pady=10)
        
        # Start update loop
        self.update_gui()
        self.root.mainloop()
        
    def draw_wheel_rotations(self):
        """Draw wheel rotation visualization"""
        canvas = self.wheel_canvas
        canvas.delete("all")
        
        # Draw robot body outline
        canvas.create_rectangle(150, 50, 250, 150, outline='gray')
        
        # Draw and rotate wheels
        wheel_positions = [
            (250, 50, "FR"),  # Front Right
            (150, 50, "FL"),  # Front Left
            (250, 150, "BR"), # Back Right
            (150, 150, "BL")  # Back Left
        ]
        
        for i, (x, y, name) in enumerate(wheel_positions):
            # Calculate rotation angle based on encoder counts
            angle = (self.encoder_counts[i] % self.ticks_per_rev) * (360 / self.ticks_per_rev)
            
            # Draw wheel
            self.draw_rotating_wheel(canvas, x, y, angle)
            canvas.create_text(x, y-20, text=name)
            
    def draw_rotating_wheel(self, canvas, x, y, angle):
        """Draw a single wheel with rotation indicator"""
        wheel_size = 20
        canvas.create_oval(x-wheel_size, y-wheel_size, x+wheel_size, y+wheel_size, 
                         outline='black')
        
        # Draw rotation indicator line
        rad = math.radians(angle)
        line_x = x + wheel_size * math.cos(rad)
        line_y = y + wheel_size * math.sin(rad)
        canvas.create_line(x, y, line_x, line_y, fill='red', width=2)
        
    def update_gui(self):
        try:
            # Update status
            self.status_label.config(text=f"Status: {self.last_status}")
            
            # Update speed bars and labels
            linear_pct = (abs(self.linear_speed) / 1.0) * 100
            angular_pct = (abs(self.angular_speed) / 0.1) * 100  # Changed to 0.1 for angular
            
            self.linear_speed_bar['value'] = linear_pct
            self.angular_speed_bar['value'] = angular_pct
            self.linear_speed_label.config(text=f"{self.linear_speed:.2f} m/s")
            self.angular_speed_label.config(text=f"{self.angular_speed:.2f} rad/s")
            
            # Update threshold bars and labels
            self.speed_threshold_bar['value'] = self.speed_threshold * 100
            self.steering_threshold_bar['value'] = self.steering_threshold * 100
            self.speed_threshold_label.config(text=f"{self.speed_threshold*100:.0f}%")
            self.steering_threshold_label.config(text=f"{self.steering_threshold*100:.0f}%")
            
            # Update turbo indicator
            self.turbo_label.config(foreground='red' if self.turbo_active else 'grey')
            
            # Update stick visualization
            self.stick_canvas.delete("all")
            
            # Draw left stick (with inverted visualization)
            self.draw_stick(100, 100, 0, -self.left_stick_x, "Left Stick\n(Rotation)")
            
            # Draw right stick
            self.draw_stick(300, 100, self.right_stick_y, 0, "Right Stick\n(Forward/Back)")

            # Update encoder displays
            for i in range(4):
                count = self.encoder_counts[i]
                rotations = count / self.ticks_per_rev
                self.encoder_labels[i].config(text=f"{count}")
                self.rotation_labels[i].config(text=f"{rotations:.1f} rot")

            # Update wheel rotation visualization
            self.draw_wheel_rotations()
            
            # Schedule next update
            self.root.after(50, self.update_gui)
            
        except Exception as e:
            self.get_logger().error(f'GUI update error: {str(e)}')
            
    def draw_stick(self, center_x, center_y, y_val, x_val, label):
        """Draw joystick visualization"""
        # Draw border
        self.stick_canvas.create_oval(center_x-50, center_y-50, 
                                    center_x+50, center_y+50, 
                                    outline='black')
        
        # Draw stick position
        stick_x = center_x + (x_val * 40)
        stick_y = center_y - (y_val * 40)
        self.stick_canvas.create_oval(stick_x-5, stick_y-5, 
                                    stick_x+5, stick_y+5, 
                                    fill='red')
        
        # Draw label
        self.stick_canvas.create_text(center_x, center_y+70, 
                                    text=label)

def main(args=None):
    rclpy.init(args=args)
    node = ControlVisualNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()