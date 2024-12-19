#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import Bool, Float32
import sys
from time import sleep

class XboxControl(Node):
    def __init__(self):
        super().__init__('xbox_control')
        
        # Publishers for all topics
        self.desired_vel_pub = self.create_publisher(Float32, 'desired_vel', 10)
        self.desired_steering_pub = self.create_publisher(Float32, 'desired_steering', 10)
        self.ebrake_pub = self.create_publisher(Bool, 'ebrake', 10)
        self.turbo_pub = self.create_publisher(Bool, 'turbo', 10)
        self.max_vel_pub = self.create_publisher(Float32, 'max_vel', 10)
        self.max_steering_pub = self.create_publisher(Float32, 'max_steering', 10)
        self.heartbeat_pub = self.create_publisher(Bool, 'heartbeat', 10)
        
        # Initialize pygame without display
        pygame.init()
        pygame.joystick.init()
        
        # Initialize controller parameters
        self.max_vel = 50.0
        self.max_steering = 50.0
        self.vel_increment = 5.0
        self.steering_increment = 5.0
        self.joystick = None
        
        # Connect to controller
        self.connect_controller()
        
        # Timers
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz control loop
        self.check_timer = self.create_timer(1.0, self.check_controller)  # Check controller every 1s
        self.heartbeat_timer = self.create_timer(0.1, self.publish_heartbeat)  # 10Hz heartbeat
        
    def publish_heartbeat(self):
        """Publish controller connection status"""
        msg = Bool()
        msg.data = bool(self.joystick and self.joystick.get_init())
        self.heartbeat_pub.publish(msg)

    def connect_controller(self):
        """Try to connect to the Xbox controller"""
        try:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info('Xbox controller connected')
                return True
            return False
        except pygame.error as e:
            self.get_logger().error(f'Controller error: {str(e)}')
            return False
            
    def check_controller(self):
        """Periodically check controller connection"""
        if not self.joystick or not self.joystick.get_init():
            pygame.joystick.quit()
            pygame.joystick.init()
            if not self.connect_controller():
                self.get_logger().warn('No controller connected. Retrying...')
                self.publish_zero_states()

    def publish_zero_states(self):
        """Publish zero values for all states"""
        # Create all messages
        zero_float = Float32()
        zero_float.data = 0.0
        false_bool = Bool()
        false_bool.data = False
        
        # Publish current max values
        max_vel_msg = Float32()
        max_vel_msg.data = self.max_vel
        self.max_vel_pub.publish(max_vel_msg)
        
        max_steering_msg = Float32()
        max_steering_msg.data = self.max_steering
        self.max_steering_pub.publish(max_steering_msg)
        
        # Publish zeros for velocities
        self.desired_vel_pub.publish(zero_float)
        self.desired_steering_pub.publish(zero_float)
        
        # Publish false for states
        self.ebrake_pub.publish(false_bool)
        self.turbo_pub.publish(false_bool)

    def control_loop(self):
        """Main control loop"""
        if not self.joystick or not self.joystick.get_init():
            return
            
        try:
            pygame.event.pump()
            
            # Get joystick values
            vel = -self.joystick.get_axis(1)  # Right stick Y (inverted)
            steering = self.joystick.get_axis(0)  # Left stick X
            
            # Get button states - explicitly convert to bool
            turbo_active = bool(self.joystick.get_button(5))  # RB
            brake_active = bool(self.joystick.get_button(1))  # B button
            
            # D-pad for max velocity/steering adjustment
            hat_x, hat_y = self.joystick.get_hat(0)
            
            # Update max values
            if hat_y == 1:  # Up
                self.max_vel = min(100.0, self.max_vel + self.vel_increment)
                self.get_logger().info(f'Max velocity: {self.max_vel}')
            elif hat_y == -1:  # Down
                self.max_vel = max(0.0, self.max_vel - self.vel_increment)
                self.get_logger().info(f'Max velocity: {self.max_vel}')
                
            if hat_x == 1:  # Right
                self.max_steering = min(100.0, self.max_steering + self.steering_increment)
                self.get_logger().info(f'Max steering: {self.max_steering}')
            elif hat_x == -1:  # Left
                self.max_steering = max(0.0, self.max_steering - self.steering_increment)
                self.get_logger().info(f'Max steering: {self.max_steering}')
            
            # Publish all states
            # Publish max values
            max_vel_msg = Float32()
            max_vel_msg.data = float(self.max_vel)  # Explicit float conversion
            self.max_vel_pub.publish(max_vel_msg)
            
            max_steering_msg = Float32()
            max_steering_msg.data = float(self.max_steering)  # Explicit float conversion
            self.max_steering_pub.publish(max_steering_msg)
            
            # Publish turbo and brake states
            turbo_msg = Bool()
            turbo_msg.data = turbo_active  # Now properly bool
            self.turbo_pub.publish(turbo_msg)
            
            brake_msg = Bool()
            brake_msg.data = brake_active  # Now properly bool
            self.ebrake_pub.publish(brake_msg)
            
            # Calculate and publish velocities
            if brake_active:
                self.publish_zero_states()
            else:
                vel_multiplier = 2.0 if turbo_active else 1.0
                
                # Calculate final velocities
                desired_vel = float(vel * self.max_vel * vel_multiplier)  # Explicit float conversion
                desired_steering = float(steering * self.max_steering)  # Explicit float conversion
                
                # Publish desired velocities
                vel_msg = Float32()
                vel_msg.data = desired_vel
                self.desired_vel_pub.publish(vel_msg)
                
                steering_msg = Float32()
                steering_msg.data = desired_steering
                self.desired_steering_pub.publish(steering_msg)
                
        except pygame.error as e:
            self.get_logger().error(f'Controller error: {str(e)}')
            self.joystick = None  # Force reconnection attempt

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