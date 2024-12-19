#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from collections import deque
import time

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        
    def update(self, value):
        self.values.append(value)
        return self.get_average()
        
    def get_average(self):
        if not self.values:
            return 0.0
        return sum(self.values) / len(self.values)

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        # Constants
        self.WHEEL_DIAMETER = 0.068  # 127mm wheels
        self.WHEEL_CIRCUMFERENCE = np.pi * self.WHEEL_DIAMETER
        self.ENCODER_CPR = 1004.19/2  # Encoder counts per revolution
        
        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # State variables for each wheel
        self.last_encoder_counts = [0, 0, 0, 0]  # FR, FL, BR, BL
        self.last_encoder_time = time.time()
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Moving average filter for car velocity
        self.velocity_filter = MovingAverageFilter(window_size=5)
        
        # Publishers
        self.car_velocity_pub = self.create_publisher(
            Float32, 
            '/car/velocity', 
            self.qos
        )
        self.wheel_velocities_pub = self.create_publisher(
            Float32MultiArray, 
            '/wheel_velocities', 
            self.qos
        )
        
        # Subscriber
        self.create_subscription(
            Int32MultiArray,
            'wheel_encoders',
            self.encoder_callback,
            self.qos
        )
        
        # Timer for publishing (50Hz)
        self.create_timer(0.02, self.publish_velocities)
        
        self.get_logger().info('Velocity Publisher initialized')
        
    def calculate_velocity(self, current_counts, wheel_index, current_time):
        """Calculate velocity for a single wheel"""
        dt = current_time - self.last_encoder_time
        
        if dt > 0:
            count_diff = current_counts - self.last_encoder_counts[wheel_index]
            counts_per_sec = count_diff / dt
            rps = counts_per_sec / self.ENCODER_CPR
            velocity = rps * self.WHEEL_CIRCUMFERENCE
            
            # Basic sanity check
            if abs(velocity) < 10.0:  # Max reasonable velocity
                return velocity
                
        return self.wheel_velocities[wheel_index]  # Return last valid velocity if calculation invalid
        
    def encoder_callback(self, msg):
        """Process encoder updates and calculate velocities"""
        current_time = time.time()
        
        # Update velocities for each wheel
        for i in range(min(len(msg.data), 4)):
            self.wheel_velocities[i] = self.calculate_velocity(msg.data[i], i, current_time)
            self.last_encoder_counts[i] = msg.data[i]
            
        self.last_encoder_time = current_time
        
    def publish_velocities(self):
        """Publish both car velocity and wheel velocities"""
        # Calculate average car velocity (ignoring zeros)
        valid_velocities = [v for v in self.wheel_velocities if abs(v) > 0.001]
        if valid_velocities:
            avg_velocity = sum(valid_velocities) / len(valid_velocities)
        else:
            avg_velocity = 0.0
            
        # Apply moving average filter to car velocity
        filtered_velocity = self.velocity_filter.update(avg_velocity)
        
        # Publish car velocity
        car_vel_msg = Float32()
        car_vel_msg.data = float(filtered_velocity)
        self.car_velocity_pub.publish(car_vel_msg)
        
        # Publish wheel velocities
        wheel_vel_msg = Float32MultiArray()
        wheel_vel_msg.data = [float(v) for v in self.wheel_velocities]
        self.wheel_velocities_pub.publish(wheel_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()