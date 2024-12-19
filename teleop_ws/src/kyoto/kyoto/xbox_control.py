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
        self.vel_pub = self.create_publisher(Float32, 'desired_vel', self.qos)
        self.steering_pub = self.create_publisher(Float32, 'desired_steering', self.qos)
        self.ebrake_pub = self.create_publisher(Bool, 'ebrake', self.qos)
        self.turbo_pub = self.create_publisher(Bool, 'turbo', self.qos)
        self.max_vel_pub = self.create_publisher(Float32, 'max_vel', self.qos)
        self.max_steering_pub = self.create_publisher(Float32, 'max_steering', self.qos)
        self.heartbeat_pub = self.create_publisher(Bool, 'heartbeat', self.qos)
        
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()
        
        # Controller mappings (based on actual testing)
        self.AXIS_LEFT_X = 0      # Left stick X: -1 (left) to 1 (right)
        self.AXIS_RIGHT_Y = 4     # Right stick Y: -1 (up) to 1 (down)
        self.BUTTON_B = 1         # B button
        self.BUTTON_START = 7     # Right back button
        
        # Control parameters
        self.max_vel = 1.0        # Initial max velocity (m/s)
        self.max_steering = 0.2    # Initial max steering (rad/s)
        self.vel_increment = 0.01   # Speed adjustment increment (m/s)
        self.steering_increment = 0.001  # Steering adjustment increment (rad/s)
        self.deadzone = 0.05      # Joystick deadzone
        
        # State variables
        self.ebrake_active = False  # E-brake state
        self.last_b_button = False  # For tracking button state change
        
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
            turbo = bool(self.joystick.get_button(self.BUTTON_START))
            
            # Handle e-brake toggle
            if current_b_button and not self.last_b_button:  # Button press detected
                self.ebrake_active = not self.ebrake_active  # Toggle state
                self.get_logger().info(f'E-brake {"activated" if self.ebrake_active else "deactivated"}')
            self.last_b_button = current_b_button
            
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
                # Velocity control (-max_vel to +max_vel)
                vel_multiplier = 2.0 if turbo else 1.0
                desired_vel = float(velocity * self.max_vel * vel_multiplier)
                vel_msg = Float32()
                vel_msg.data = desired_vel
                self.vel_pub.publish(vel_msg)
                
                # Steering control (-max_steering to +max_steering)
                desired_steering = float(steering * self.max_steering)
                steering_msg = Float32()
                steering_msg.data = desired_steering
                self.steering_pub.publish(steering_msg)
            else:
                # When e-brake is active, publish zero values
                zero_msg = Float32()
                zero_msg.data = 0.0
                self.vel_pub.publish(zero_msg)
                self.steering_pub.publish(zero_msg)
                
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