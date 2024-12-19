#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import pygame
import sys

class XboxControl(Node):
    def __init__(self):
        super().__init__('xbox_control')
        
        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize publishers with QoS
        # Original control topics
        self.vel_pub = self.create_publisher(Float32, 'desired_velocity', self.qos)
        self.steering_pub = self.create_publisher(Float32, 'desired_steering', self.qos)
        
        # Manual control topics
        self.manual_vel_pub = self.create_publisher(Float32, 'manual_velocity', self.qos)
        self.manual_steering_pub = self.create_publisher(Float32, 'manual_steering', self.qos)
        
        # Status topics
        self.ebrake_pub = self.create_publisher(Bool, 'ebrake', self.qos)
        self.turbo_pub = self.create_publisher(Bool, 'turbo', self.qos)
        self.max_vel_pub = self.create_publisher(Float32, 'max_vel', self.qos)
        self.max_steering_pub = self.create_publisher(Float32, 'max_steering', self.qos)
        self.heartbeat_pub = self.create_publisher(Bool, 'heartbeat', self.qos)
        
        # Control mode publishers
        self.manual_mode_pub = self.create_publisher(Bool, 'manual_mode', self.qos)
        self.auto_mode_pub = self.create_publisher(Bool, 'auto_mode', self.qos)
        
        # Return flag publisher
        self.return_pub = self.create_publisher(Bool, 'return', self.qos)
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Controller mappings (based on actual testing)
        self.AXIS_LEFT_X = 0      # Left stick X: -1 (left) to 1 (right)
        self.AXIS_RIGHT_Y = 4     # Right stick Y: -1 (up) to 1 (down)
        self.BUTTON_A = 0         # A button
        self.BUTTON_B = 1         # B button
        self.BUTTON_X = 2         # X button
        self.BUTTON_Y = 3         # Y button
        self.BUTTON_START = 7     # Right back button
        
        # Control parameters
        self.max_vel = 1.0        # Initial max velocity (m/s)
        self.max_steering = 0.2    # Initial max steering (rad/s)
        self.vel_increment = 0.01   # Speed adjustment increment (m/s)
        self.steering_increment = 0.001  # Steering adjustment increment (rad/s)
        self.deadzone = 0.05      # Joystick deadzone
        
        # State variables
        self.ebrake_active = False    # E-brake state
        self.last_b_button = False    # For tracking B button state change
        self.last_a_button = False    # For tracking A button state change
        self.last_y_button = False    # For tracking Y button state change
        self.last_x_button = False    # For tracking X button state change
        self.manual_mode = False      # Manual control mode state
        self.auto_mode = False        # Auto control mode state
        self.return_active = False    # Return flag state
        
        # Connect controller
        self.joystick = None
        self.connect_controller()
        
        # Create timers
        self.create_timer(0.02, self.control_loop)       # 50Hz main control loop
        self.create_timer(0.1, self.publish_heartbeat)   # 10Hz heartbeat
        
        self.get_logger().info('Xbox controller initialized')
        
    def connect_controller(self):
        """Connect to Xbox controller"""
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info('Controller connected')
            return True
        else:
            self.get_logger().warn('No controller found')
            return False
            
    def apply_deadzone(self, value):
        """Apply deadzone to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
        
    def publish_heartbeat(self):
        """Publish controller status"""
        msg = Bool()
        msg.data = bool(self.joystick and self.joystick.get_init())
        self.heartbeat_pub.publish(msg)
        
    def control_loop(self):
        """Main control loop"""
        if not self.joystick:
            if not self.connect_controller():
                return
                
        try:
            pygame.event.pump()
            
            # Read joystick inputs
            steering = self.apply_deadzone(self.joystick.get_axis(self.AXIS_LEFT_X))
            velocity = -self.apply_deadzone(self.joystick.get_axis(self.AXIS_RIGHT_Y))  # Inverted
            
            # Read buttons
            current_b_button = bool(self.joystick.get_button(self.BUTTON_B))
            current_a_button = bool(self.joystick.get_button(self.BUTTON_A))
            current_y_button = bool(self.joystick.get_button(self.BUTTON_Y))
            current_x_button = bool(self.joystick.get_button(self.BUTTON_X))
            turbo = bool(self.joystick.get_button(self.BUTTON_START))
            
            # Handle mode toggles
            if current_a_button and not self.last_a_button:  # A button press
                self.manual_mode = not self.manual_mode
                if self.manual_mode:
                    self.auto_mode = False  # Disable auto mode when manual is enabled
                self.get_logger().info(f'Manual mode {"enabled" if self.manual_mode else "disabled"}')
                
            if current_y_button and not self.last_y_button:  # Y button press
                self.auto_mode = not self.auto_mode
                if self.auto_mode:
                    self.manual_mode = False  # Disable manual mode when auto is enabled
                self.get_logger().info(f'Auto mode {"enabled" if self.auto_mode else "disabled"}')
            
            # Handle return flag toggle
            if current_x_button and not self.last_x_button:  # X button press
                self.return_active = not self.return_active
                self.get_logger().info(f'Return flag {"activated" if self.return_active else "deactivated"}')
            
            # Handle e-brake toggle
            if current_b_button and not self.last_b_button:  # Button press detected
                self.ebrake_active = not self.ebrake_active  # Toggle state
                self.get_logger().info(f'E-brake {"activated" if self.ebrake_active else "deactivated"}')
            
            # Update button states
            self.last_b_button = current_b_button
            self.last_a_button = current_a_button
            self.last_y_button = current_y_button
            self.last_x_button = current_x_button
            
            # Publish control mode states
            manual_mode_msg = Bool()
            manual_mode_msg.data = self.manual_mode
            self.manual_mode_pub.publish(manual_mode_msg)
            
            auto_mode_msg = Bool()
            auto_mode_msg.data = self.auto_mode
            self.auto_mode_pub.publish(auto_mode_msg)
            
            # Publish return flag state
            return_msg = Bool()
            return_msg.data = self.return_active
            self.return_pub.publish(return_msg)
            
            # Publish e-brake state
            ebrake_msg = Bool()
            ebrake_msg.data = self.ebrake_active
            self.ebrake_pub.publish(ebrake_msg)
            
            # Read D-pad
            hat_x, hat_y = self.joystick.get_hat(0)
            
            # Update limits based on D-pad
            if hat_y != 0:  # Up/Down for velocity limit
                self.max_vel = min(5.0, max(0.0, 
                    self.max_vel + (self.vel_increment * hat_y)))
                self.get_logger().info(f'Max velocity: {self.max_vel:.2f} m/s')
                
            if hat_x != 0:  # Left/Right for steering limit
                self.max_steering = min(0.5, max(0.0, 
                    self.max_steering + (self.steering_increment * hat_x)))
                self.get_logger().info(f'Max steering: {self.max_steering:.2f} rad/s')
            
            # Publish max values
            max_vel_msg = Float32()
            max_vel_msg.data = float(self.max_vel)
            self.max_vel_pub.publish(max_vel_msg)
            
            max_steering_msg = Float32()
            max_steering_msg.data = float(self.max_steering)
            self.max_steering_pub.publish(max_steering_msg)
            
            # Publish turbo state
            turbo_msg = Bool()
            turbo_msg.data = turbo
            self.turbo_pub.publish(turbo_msg)
            
            # Calculate and publish control values
            if not self.ebrake_active:
                # Velocity and steering multiplier
                vel_multiplier = 2.0 if turbo else 1.0
                
                # Calculate desired values
                desired_vel = float(velocity * self.max_vel * vel_multiplier)
                desired_steering = float(steering * self.max_steering)
                
                # Create messages
                vel_msg = Float32()
                steering_msg = Float32()
                manual_vel_msg = Float32()
                manual_steering_msg = Float32()
                
                # Set message data
                vel_msg.data = desired_vel
                steering_msg.data = desired_steering
                manual_vel_msg.data = desired_vel
                manual_steering_msg.data = desired_steering
                
                # Always publish to desired topics
                self.vel_pub.publish(vel_msg)
                self.steering_pub.publish(steering_msg)
                
                # Publish to manual topics only in manual mode
                if self.manual_mode:
                    self.manual_vel_pub.publish(manual_vel_msg)
                    self.manual_steering_pub.publish(manual_steering_msg)
            else:
                # When e-brake is active, publish zero values to all control topics
                zero_msg = Float32()
                zero_msg.data = 0.0
                self.vel_pub.publish(zero_msg)
                self.steering_pub.publish(zero_msg)
                self.manual_vel_pub.publish(zero_msg)
                self.manual_steering_pub.publish(zero_msg)
                
        except pygame.error as e:
            self.get_logger().error(f'Controller error: {str(e)}')
            self.joystick = None

def main(args=None):
    rclpy.init(args=args)
    node = XboxControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()