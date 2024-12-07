#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import time

class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control')
        print("Starting Xbox Controller Node...")
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_speed', 1.0),       # Max linear speed (m/s)
                ('angular_speed', 0.1),      # Max angular speed (rad/s)
                ('deadzone', 0.1),           # Joystick deadzone
                ('turbo_multiplier', 2.0),   # Speed multiplier when turbo is active
                ('update_rate', 50.0)        # Control update rate in Hz
            ]
        )

        # Get parameter values
        self.base_linear_speed = self.get_parameter('linear_speed').value
        self.base_angular_speed = self.get_parameter('angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        self.update_rate = self.get_parameter('update_rate').value
        print(f"Parameters loaded - Base speeds: Linear={self.base_linear_speed}, Angular={self.base_angular_speed}")

        # Threshold values (0.0 to 1.0)
        self.speed_threshold = 1.0
        self.steering_threshold = 1.0
        self.threshold_step = 0.05  # 5% change per key press

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'controller_status', 10)
        self.speed_pub = self.create_publisher(Float32MultiArray, 'current_speeds', 10)
        self.threshold_pub = self.create_publisher(Float32MultiArray, 'control_thresholds', 10)
        print("Publishers created")

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        print("Joy subscriber created")

        # Xbox controller mappings
        self.axis_mapping = {
            'right_stick_y': 4,   # Forward/back (swapped)
            'left_stick_x': 0,    # Rotation (swapped)
        }

        self.button_mapping = {
            'A': 0,
            'B': 1,               # Emergency stop
            'X': 2,
            'Y': 3,
            'LB': 4,
            'RB': 5,             # Turbo
            'start': 6,
            'back': 7,
            'dpad_up': 7,       
            'dpad_down': 8,      
            'dpad_left': 9,     
            'dpad_right': 10    
        }

        # State variables
        self.active = True
        self.last_command_time = time.time()
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        # Timer for publishing thresholds
        self.timer = self.create_timer(0.1, self.publish_thresholds)

        self.get_logger().info('Xbox controller node initialized')
        self.publish_status('Controller initialized')
        print("Initialization complete!")

    def publish_thresholds(self):
        """Publish current threshold values"""
        msg = Float32MultiArray()
        msg.data = [self.speed_threshold, self.steering_threshold]
        self.threshold_pub.publish(msg)

    def publish_status(self, message):
        """Publish controller status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def publish_speeds(self):
        """Publish current speed values"""
        msg = Float32MultiArray()
        msg.data = [self.current_linear_speed, self.current_angular_speed]
        self.speed_pub.publish(msg)

    def apply_deadzone(self, value):
        """Apply deadzone to joystick values"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        normalized = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * normalized

    def joy_callback(self, msg):
        """Process joystick inputs and generate movement commands"""
        print("\n--- Joy Callback Start ---")
        
        if not self.active:
            print("Controller not active!")
            return

        # Print raw joystick values
        print(f"Raw Right stick Y (forward/back): {msg.axes[self.axis_mapping['right_stick_y']]}")
        print(f"Raw Left stick X (rotation): {msg.axes[self.axis_mapping['left_stick_x']]}")

        # Check emergency stop (B button)
        if msg.buttons[self.button_mapping['B']]:
            print("Emergency stop pressed!")
            self.emergency_stop()
            return

        # Print D-pad length for debugging
        print(f"Number of buttons: {len(msg.buttons)}")

        # Handle threshold adjustments with D-pad
        try:
            if msg.buttons[self.button_mapping['dpad_up']]:
                self.speed_threshold = min(1.0, self.speed_threshold + self.threshold_step)
                print(f"Speed threshold increased to: {self.speed_threshold}")
            elif msg.buttons[self.button_mapping['dpad_down']]:
                self.speed_threshold = max(0.1, self.speed_threshold - self.threshold_step)
                print(f"Speed threshold decreased to: {self.speed_threshold}")
            elif msg.buttons[self.button_mapping['dpad_right']]:
                self.steering_threshold = min(1.0, self.steering_threshold + self.threshold_step)
                print(f"Steering threshold increased to: {self.steering_threshold}")
            elif msg.buttons[self.button_mapping['dpad_left']]:
                self.steering_threshold = max(0.1, self.steering_threshold - self.threshold_step)
                print(f"Steering threshold decreased to: {self.steering_threshold}")
        except IndexError:
            print(f"Warning: D-pad button index out of range. Available buttons: {len(msg.buttons)}")

        # Get turbo multiplier if RB is pressed
        speed_multiplier = self.turbo_multiplier if msg.buttons[self.button_mapping['RB']] else 1.0
        if speed_multiplier > 1.0:
            print("Turbo active!")

        # Create Twist message
        cmd = Twist()

        # Apply deadzone and get movement values (swapped sticks)
        forward_back = self.apply_deadzone(msg.axes[self.axis_mapping['right_stick_y']])
        rotation = self.apply_deadzone(msg.axes[self.axis_mapping['left_stick_x']])
        print(f"After deadzone - Forward/Back: {forward_back}, Rotation: {rotation}")

        # Apply thresholds and calculate speeds
        self.current_linear_speed = forward_back * self.base_linear_speed * speed_multiplier * self.speed_threshold
        self.current_angular_speed = rotation * self.base_angular_speed * speed_multiplier * self.steering_threshold
        print(f"Calculated speeds - Linear: {self.current_linear_speed}, Angular: {self.current_angular_speed}")

        # Set command velocities
        cmd.linear.x = self.current_linear_speed
        cmd.angular.z = self.current_angular_speed

        # Publish command and speeds
        self.cmd_vel_pub.publish(cmd)
        self.publish_speeds()
        self.last_command_time = time.time()
        print("Published cmd_vel and speeds!")
        print("--- Joy Callback End ---\n")

    def emergency_stop(self):
        """Handle emergency stop"""
        print("Emergency stop activated!")
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.publish_speeds()
        self.publish_status('Emergency Stop Activated')

def main(args=None):
    print("Starting main function...")
    rclpy.init(args=args)
    node = XboxControlNode()
    print("Node created, starting spin...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
        node.emergency_stop()
        node.publish_status('Controller node shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete")

if __name__ == '__main__':
    main()