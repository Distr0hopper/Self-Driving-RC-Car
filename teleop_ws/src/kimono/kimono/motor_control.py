#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32MultiArray
import numpy as np
import time

class PIDController:
    def __init__(self, Kp=20.0, Ki=900.0, Kd=0.50, sample_time=0.02):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # Anti-windup limits
        self.output_limits = (-255, 255)
        self.integral_limits = (-10, 10)
        
        # Derivative filter
        self.alpha = 0.3
        self.filtered_derivative = 0.0
        
    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        
    def compute(self, setpoint, measured_value, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = setpoint - measured_value
            return 0
            
        dt = current_time - self.last_time
        if dt < self.sample_time:
            return None
            
        error = setpoint - measured_value
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup
        self.integral = np.clip(self.integral + error * dt, 
                              self.integral_limits[0], 
                              self.integral_limits[1])
        I = self.Ki * self.integral
        
        # Derivative term with low-pass filter
        if dt > 0:
            current_derivative = (error - self.last_error) / dt
            self.filtered_derivative = (self.alpha * current_derivative + 
                                     (1 - self.alpha) * self.filtered_derivative)
        else:
            self.filtered_derivative = 0
            
        D = self.Kd * self.filtered_derivative
        
        # Calculate total output
        output = P + I + D
        
        # Clip output to limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Create left and right PIDs
        self.left_pid = PIDController()
        self.right_pid = PIDController()
        
        # Subscribers
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, 10)
        self.create_subscription(Float32, 'desired_steering', self.steering_callback, 10)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, 10)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, 10)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, 10)
        
        # Publisher for motor commands [FR, FL, BR, BL]
        self.motor_pub = self.create_publisher(Int32MultiArray, 'motor_pwm', 10)
        
        # Control state
        self.desired_vel = 0.0
        self.desired_steering = 0.0
        self.ebrake_active = False
        self.turbo_active = False
        
        # Encoder processing
        self.last_encoder_counts = None
        self.last_encoder_time = None
        self.reading_count = 0
        self.encoder_cpr = 204.19  # From your config
        self.wheel_circumference = np.pi * 0.244  # From your config
        
        # Control loop timer (50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Motor controller initialized')

    def vel_callback(self, msg):
        self.desired_vel = msg.data

    def steering_callback(self, msg):
        self.desired_steering = msg.data

    def ebrake_callback(self, msg):
        self.ebrake_active = msg.data
        if self.ebrake_active:
            self.left_pid.reset()
            self.right_pid.reset()

    def turbo_callback(self, msg):
        self.turbo_active = msg.data

    def get_wheel_velocities(self, counts, current_time):
        """Calculate wheel velocities from encoder counts"""
        if self.last_encoder_counts is None:
            self.last_encoder_counts = counts
            self.last_encoder_time = current_time
            self.reading_count = 0
            return None, None
        
        dt = current_time - self.last_encoder_time
        if dt < 0.02:  # Minimum sample time
            return None, None
            
        self.reading_count += 1
        
        # Skip first 2 readings to let velocity calculation stabilize
        if self.reading_count <= 2:
            self.last_encoder_counts = counts
            self.last_encoder_time = current_time
            return None, None
            
        # Calculate velocities for left and right sides
        left_counts = (counts[1] + counts[3]) / 2  # Average of FL and BL
        right_counts = (counts[0] + counts[2]) / 2  # Average of FR and BR
        
        left_diff = left_counts - (self.last_encoder_counts[1] + self.last_encoder_counts[3]) / 2
        right_diff = right_counts - (self.last_encoder_counts[0] + self.last_encoder_counts[2]) / 2
        
        # Convert to linear velocity
        left_vel = (left_diff / dt) / self.encoder_cpr * self.wheel_circumference
        right_vel = (right_diff / dt) / self.encoder_cpr * self.wheel_circumference
        
        self.last_encoder_counts = counts
        self.last_encoder_time = current_time
        
        return left_vel, right_vel

    def encoder_callback(self, msg):
        current_time = time.time()
        left_vel, right_vel = self.get_wheel_velocities(msg.data, current_time)
        
        if left_vel is not None and right_vel is not None:
            # Calculate desired wheel velocities from desired velocity and steering
            left_setpoint = self.desired_vel + self.desired_steering
            right_setpoint = self.desired_vel - self.desired_steering
            
            if self.turbo_active:
                left_setpoint *= 2.0
                right_setpoint *= 2.0
            
            # Apply PIDs if not in e-brake
            if not self.ebrake_active:
                left_pwm = self.left_pid.compute(left_setpoint, left_vel, current_time)
                right_pwm = self.right_pid.compute(right_setpoint, right_vel, current_time)
                
                if left_pwm is not None and right_pwm is not None:
                    # Invert right side PWM
                    right_pwm = -right_pwm
                    
                    # Create motor command [FR, FL, BR, BL]
                    motor_msg = Int32MultiArray()
                    motor_msg.data = [
                        int(right_pwm),  # FR (inverted)
                        int(left_pwm),   # FL
                        int(right_pwm),  # BR (inverted)
                        int(left_pwm)    # BL
                    ]
                    self.motor_pub.publish(motor_msg)
            else:
                # Send zero PWM when e-brake is active
                motor_msg = Int32MultiArray()
                motor_msg.data = [0, 0, 0, 0]
                self.motor_pub.publish(motor_msg)

    def control_loop(self):
        # This is just a heartbeat to ensure we're still sending commands
        # The actual control happens in encoder_callback
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = MotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors
        stop_msg = Int32MultiArray()
        stop_msg.data = [0, 0, 0, 0]
        controller.motor_pub.publish(stop_msg)
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()