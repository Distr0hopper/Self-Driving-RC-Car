#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import time
from collections import deque

# Configuration constants
MAX_PWM = 128
WHEEL_DIAMETER = 0.127
WHEEL_CIRCUMFERENCE = np.pi * WHEEL_DIAMETER
ENCODER_CPR = 204.19/2

class MovingAverageFilter:
    def __init__(self, window_size=3):  # Reduced window size
        self.window_size = window_size
        self.values = deque(maxlen=window_size)
        
    def update(self, value):
        self.values.append(value)
        return self.get_average()
        
    def get_average(self):
        if not self.values:
            return 0.0
        return sum(self.values) / len(self.values)

class PIDController:
    def __init__(self, Kp=40.0, Ki=80.0, Kd=0.2, sample_time=0.02):  # Increased gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        
        # State variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        self.last_output = 0.0
        
        # Anti-windup limits
        self.output_limits = (-MAX_PWM, MAX_PWM)
        self.integral_limits = (-25, 25)
        
        # Derivative filter
        self.alpha = 0.1  # Increased alpha for more responsive derivative
        
        # Deadband for steady state
        self.deadband = 0.01  # Reduced deadband
        
    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        self.last_output = 0.0
        
    def compute(self, setpoint, measured_value, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = setpoint - measured_value
            return 0
            
        dt = current_time - self.last_time
        if dt < 1e-6:
            return self.last_output
            
        # Calculate error
        error = setpoint - measured_value
        
        # Apply deadband only when very close to target and stable
        if abs(error) < self.deadband and abs(self.filtered_derivative) < 0.05:
            error = 0
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term with improved anti-windup
        if abs(error) < 1.0:  # Increased range for integral action
            self.integral = np.clip(
                self.integral + error * dt,
                self.integral_limits[0],
                self.integral_limits[1]
            )
        I = self.Ki * self.integral
        
        # Derivative term with improved filtering
        current_derivative = (error - self.last_error) / dt
        self.filtered_derivative = (self.alpha * current_derivative + 
                                 (1 - self.alpha) * self.filtered_derivative)
        D = self.Kd * self.filtered_derivative
        
        # Calculate total output
        output = np.clip(P + I + D, 
                        self.output_limits[0], 
                        self.output_limits[1])
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output

class SimpleMotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.fr_pid = PIDController()
        
        # Improved state tracking
        self.last_encoder_counts = 0
        self.last_encoder_time = time.time()
        self.current_velocity = 0.0
        self.last_valid_velocity = 0.0
        self.last_pwm = 0
        
        # Reduced filtering
        self.velocity_filter = MovingAverageFilter(window_size=3)
        self.pwm_filter = MovingAverageFilter(window_size=2)
        
        # Subscribers
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, self.qos)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, self.qos)
        
        self.motor_pub = self.create_publisher(Int32MultiArray, 'motor_pwm', self.qos)
        
        # Control state
        self.desired_vel = 0.0
        self.ebrake_active = False
        self.turbo_active = False
        
        # Create control loop timer (50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Motor Controller initialized')
        
    def vel_callback(self, msg):
        self.desired_vel = msg.data
        
    def ebrake_callback(self, msg):
        self.ebrake_active = msg.data
        if self.ebrake_active:
            self.fr_pid.reset()
            self.publish_motor_commands([0, 0, 0, 0])
            
    def turbo_callback(self, msg):
        self.turbo_active = msg.data
        
    def publish_motor_commands(self, pwm_values):
        msg = Int32MultiArray()
        msg.data = [int(pwm) for pwm in pwm_values]
        self.motor_pub.publish(msg)
        
    def get_velocity(self, counts, current_time):
        dt = current_time - self.last_encoder_time
        
        if dt > 0:
            count_diff = counts - self.last_encoder_counts
            counts_per_sec = count_diff / dt
            rps = counts_per_sec / ENCODER_CPR
            velocity = rps * WHEEL_CIRCUMFERENCE
            
            # Sanity check on velocity
            if abs(velocity) < 10.0:
                filtered_velocity = self.velocity_filter.update(velocity)
                self.last_valid_velocity = filtered_velocity
            
            self.last_encoder_counts = counts
            self.last_encoder_time = current_time
            
            return self.last_valid_velocity
            
        return self.last_valid_velocity
        
    def encoder_callback(self, msg):
        current_time = time.time()
        if len(msg.data) > 0:
            self.current_velocity = self.get_velocity(msg.data[0], current_time)
            
    def control_loop(self):
        if self.ebrake_active:
            self.fr_pid.reset()
            self.publish_motor_commands([0, 0, 0, 0])
            return
            
        # Add explicit check for zero desired velocity
        if abs(self.desired_vel) < 0.001:  # Small threshold to account for floating point
            self.fr_pid.reset()  # Reset PID state
            self.last_pwm = 0  # Reset last PWM
            self.publish_motor_commands([0, 0, 0, 0])
            return
            
        target_velocity = self.desired_vel * (2.0 if self.turbo_active else 1.0)
        
        # Get PID output
        fr_pwm = self.fr_pid.compute(target_velocity, self.current_velocity, time.time())
        
        if fr_pwm is not None:
            # Apply PWM filtering
            filtered_pwm = self.pwm_filter.update(fr_pwm)
            
            # Limit rate of change but allow faster changes
            pwm_change = np.clip(filtered_pwm - self.last_pwm, -8, 8)
            fr_pwm = self.last_pwm + pwm_change
            self.last_pwm = fr_pwm
            
            motor_commands = [
                -int(fr_pwm),  # FR (inverted)
                0,            # FL
                0,            # BR
                0             # BL
            ]
            self.publish_motor_commands(motor_commands)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleMotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.publish_motor_commands([0, 0, 0, 0])
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()