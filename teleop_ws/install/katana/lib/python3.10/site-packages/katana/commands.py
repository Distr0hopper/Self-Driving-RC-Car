#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class CommandManager(Node):
    def __init__(self):
        super().__init__('command_manager')
        
        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers for final commands
        self.vel_pub = self.create_publisher(Float32, 'desired_velocity', self.qos)
        self.steering_pub = self.create_publisher(Float32, 'desired_steering', self.qos)
        
        # Subscribers for control modes
        self.create_subscription(Bool, 'manual_mode', self.manual_mode_callback, self.qos)
        self.create_subscription(Bool, 'auto_mode', self.auto_mode_callback, self.qos)
        self.create_subscription(Bool, 'return', self.return_callback, self.qos)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        
        # Subscribers for manual commands
        self.create_subscription(Float32, 'manual_velocity', self.manual_vel_callback, self.qos)
        self.create_subscription(Float32, 'manual_steering', self.manual_steering_callback, self.qos)
        
        # Subscribers for auto commands
        self.create_subscription(Float32, 'auto_velocity', self.auto_vel_callback, self.qos)
        self.create_subscription(Float32, 'auto_steering', self.auto_steering_callback, self.qos)
        
        # Subscribers for return commands
        self.create_subscription(Float32, 'return_velocity', self.return_vel_callback, self.qos)
        self.create_subscription(Float32, 'return_steering', self.return_steering_callback, self.qos)
        
        # State variables
        self.manual_mode = False
        self.auto_mode = False
        self.return_active = False
        self.ebrake_active = False
        
        # Command storage
        self.manual_velocity = 0.0
        self.manual_steering = 0.0
        self.auto_velocity = 0.0
        self.auto_steering = 0.0
        self.return_velocity = 0.0
        self.return_steering = 0.0
        
        # Current commands to publish
        self.current_velocity = 0.0
        self.current_steering = 0.0
        
        # Create timer for 50Hz control loop
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Command manager initialized')
        
    def manual_mode_callback(self, msg):
        """Handle manual mode changes"""
        self.manual_mode = msg.data
        if msg.data:
            self.auto_mode = False
            self.return_active = False
            self.get_logger().info('Switched to manual mode')
            
    def auto_mode_callback(self, msg):
        """Handle auto mode changes"""
        if not self.return_active:  # Only allow auto mode if return is not active
            self.auto_mode = msg.data
            if msg.data:
                self.manual_mode = False
                self.get_logger().info('Switched to auto mode')
            
    def return_callback(self, msg):
        """Handle return mode changes"""
        self.return_active = msg.data
        if msg.data:
            self.manual_mode = False
            self.auto_mode = False
            self.get_logger().info('Switched to return mode')
            
    def ebrake_callback(self, msg):
        """Handle ebrake state"""
        self.ebrake_active = msg.data
        if msg.data:
            self.current_velocity = 0.0
            self.current_steering = 0.0
            
    def manual_vel_callback(self, msg):
        """Store manual velocity command"""
        self.manual_velocity = msg.data
        
    def manual_steering_callback(self, msg):
        """Store manual steering command"""
        self.manual_steering = msg.data
        
    def auto_vel_callback(self, msg):
        """Store auto velocity command"""
        self.auto_velocity = msg.data
        
    def auto_steering_callback(self, msg):
        """Store auto steering command"""
        self.auto_steering = msg.data
        
    def return_vel_callback(self, msg):
        """Store return velocity command"""
        self.return_velocity = msg.data
        
    def return_steering_callback(self, msg):
        """Store return steering command"""
        self.return_steering = msg.data
        
    def update_current_commands(self):
        """Update current commands based on active mode"""
        if self.ebrake_active:
            self.current_velocity = 0.0
            self.current_steering = 0.0
        elif self.manual_mode:
            self.current_velocity = self.manual_velocity
            self.current_steering = self.manual_steering
        elif self.return_active:  # Return mode takes precedence over auto mode
            self.current_velocity = self.return_velocity
            self.current_steering = self.return_steering
        elif self.auto_mode:
            self.current_velocity = self.auto_velocity
            self.current_steering = self.auto_steering
        else:
            self.current_velocity = 0.0
            self.current_steering = 0.0
            
    def publish_commands(self):
        """Publish current commands at 50Hz"""
        # Publish velocity
        vel_msg = Float32()
        vel_msg.data = self.current_velocity
        self.vel_pub.publish(vel_msg)
        
        # Publish steering
        steering_msg = Float32()
        steering_msg.data = self.current_steering
        self.steering_pub.publish(steering_msg)
        
    def control_loop(self):
        """Main control loop running at 50Hz"""
        self.update_current_commands()
        self.publish_commands()

def main(args=None):
    rclpy.init(args=args)
    node = CommandManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure zero commands on shutdown
        vel_msg = Float32()
        steering_msg = Float32()
        vel_msg.data = 0.0
        steering_msg.data = 0.0
        node.vel_pub.publish(vel_msg)
        node.steering_pub.publish(steering_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()