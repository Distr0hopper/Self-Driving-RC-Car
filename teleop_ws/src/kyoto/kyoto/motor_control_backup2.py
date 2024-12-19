#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import time
from collections import deque
from enum import Enum

# Configuration constants
MAX_PWM = 128
WHEEL_DIAMETER = 0.127  # meters
WHEEL_CIRCUMFERENCE = np.pi * WHEEL_DIAMETER
ENCODER_CPR = 204.19/2

# Tuning Parameters
STATIC_BREAK_PWM = 255     # PWM to break static friction
STATIC_BREAK_TIME = 0.1    # Time to apply break-free PWM (seconds)
VELOCITY_THRESHOLD = 0.05  # Threshold to consider wheel moving
PWM_RAMP_RATE = 5         # PWM change per control cycle

# Control Parameters
ANGULAR_SCALE = 400.0      # Scale factor for converting angular velocity to PWM

# PID Gains
KP = 40.0
KI = 80.0
KD = 0.2
INTEGRAL_LIMIT = 25.0

# Filtering
VELOCITY_FILTER_WINDOW = 3
PWM_FILTER_WINDOW = 2

class MotorState(Enum):
    STOPPED = 0
    BREAKING_STATIC = 1
    NORMAL_OPERATION = 2
    STOPPING = 3

class MovingAverageFilter:
    def __init__(self, window_size=3):
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
    def __init__(self, Kp=KP, Ki=KI, Kd=KD):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # Anti-windup limits
        self.integral_limits = (-INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        
    def reset(self):
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
    def compute(self, setpoint, measured_value, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = setpoint - measured_value
            return 0
            
        dt = current_time - self.last_time
        if dt < 1e-6:
            return 0
            
        error = setpoint - measured_value
        
        # Reset integral if direction changes
        if (error > 0 and self.last_error < 0) or (error < 0 and self.last_error > 0):
            self.integral = 0
            
        # Calculate P term
        P = self.Kp * error
        
        # Calculate I term with anti-windup
        if abs(error) < 1.0:  # Only integrate when close to target
            self.integral = np.clip(
                self.integral + error * dt,
                self.integral_limits[0],
                self.integral_limits[1]
            )
        I = self.Ki * self.integral
        
        # Calculate D term
        D = self.Kd * (error - self.last_error) / dt if dt > 0 else 0
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        # Calculate total output
        output = P + I + D
        return output

class WheelController:
    def __init__(self, invert=False):
        self.pid = PIDController()
        self.velocity_filter = MovingAverageFilter(window_size=VELOCITY_FILTER_WINDOW)
        self.last_encoder_counts = 0
        self.last_encoder_time = time.time()
        self.current_velocity = 0.0
        self.last_valid_velocity = 0.0
        self.current_pwm = 0
        self.invert = invert
        self.state = MotorState.STOPPED
        self.state_start_time = time.time()
        
    def reset(self):
        self.current_pwm = 0
        self.pid.reset()
        self.state = MotorState.STOPPED
        self.state_start_time = time.time()

    def change_state(self, new_state):
        if new_state != self.state:
            self.state = new_state
            if new_state == MotorState.STOPPED:
                self.pid.reset()
            self.state_start_time = time.time()

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize wheel controllers (FR, FL, BR, BL)
        self.wheels = [
            WheelController(invert=True),   # FR - inverted
            WheelController(invert=False),  # FL
            WheelController(invert=True),   # BR - inverted
            WheelController(invert=False)   # BL
        ]
        
        # Subscribers
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, self.qos)
        self.create_subscription(Float32, 'desired_steering', self.steering_callback, self.qos)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, self.qos)
        
        self.motor_pub = self.create_publisher(Int32MultiArray, 'motor_pwm', self.qos)
        
        # Control state
        self.desired_vel = 0.0
        self.angular_vel = 0.0
        self.ebrake_active = False
        self.turbo_active = False
        
        # Create control loop timer (50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Motor Controller initialized')
        
    def vel_callback(self, msg):
        self.desired_vel = msg.data
        if abs(msg.data) < 0.001:
            for wheel in self.wheels:
                wheel.change_state(MotorState.STOPPING)

    def steering_callback(self, msg):
        self.angular_vel = msg.data
        
    def ebrake_callback(self, msg):
        self.ebrake_active = msg.data
        if self.ebrake_active:
            for wheel in self.wheels:
                wheel.reset()
            msg = Int32MultiArray()
            msg.data = [0, 0, 0, 0]
            self.motor_pub.publish(msg)
            
    def turbo_callback(self, msg):
        self.turbo_active = msg.data
        
    def get_velocity(self, counts, wheel):
        current_time = time.time()
        dt = current_time - wheel.last_encoder_time
        
        if dt > 0:
            count_diff = counts - wheel.last_encoder_counts
            counts_per_sec = count_diff / dt
            rps = counts_per_sec / ENCODER_CPR
            velocity = rps * WHEEL_CIRCUMFERENCE
            
            if abs(velocity) < 10.0:
                filtered_velocity = wheel.velocity_filter.update(velocity)
                wheel.last_valid_velocity = filtered_velocity
            
            wheel.last_encoder_counts = counts
            wheel.last_encoder_time = current_time
            
            return wheel.last_valid_velocity
            
        return wheel.last_valid_velocity

    def update_wheel_pwm(self, wheel, target_velocity, current_time):
        # State machine for each wheel
        if wheel.state == MotorState.STOPPED:
            if abs(target_velocity) > 0.001:
                wheel.change_state(MotorState.BREAKING_STATIC)
                wheel.current_pwm = np.sign(target_velocity) * STATIC_BREAK_PWM
            else:
                wheel.current_pwm = 0
                
        elif wheel.state == MotorState.BREAKING_STATIC:
            if current_time - wheel.state_start_time > STATIC_BREAK_TIME:
                wheel.change_state(MotorState.NORMAL_OPERATION)
                
        elif wheel.state == MotorState.NORMAL_OPERATION:
            if abs(target_velocity) < 0.001:
                wheel.change_state(MotorState.STOPPING)
            else:
                # Use PID control for velocity
                pid_output = wheel.pid.compute(target_velocity, wheel.current_velocity, current_time)
                target_pwm = int(np.clip(pid_output, -MAX_PWM, MAX_PWM))
                
                # Smooth PWM changes
                pwm_diff = target_pwm - wheel.current_pwm
                if abs(pwm_diff) > PWM_RAMP_RATE:
                    wheel.current_pwm += np.sign(pwm_diff) * PWM_RAMP_RATE
                else:
                    wheel.current_pwm = target_pwm
                    
        elif wheel.state == MotorState.STOPPING:
            wheel.current_pwm = 0
            if abs(wheel.current_velocity) < VELOCITY_THRESHOLD:
                wheel.change_state(MotorState.STOPPED)
                
        # Apply PWM limits
        wheel.current_pwm = np.clip(wheel.current_pwm, -MAX_PWM, MAX_PWM)

    def encoder_callback(self, msg):
        for i, counts in enumerate(msg.data[:4]):
            self.wheels[i].current_velocity = self.get_velocity(counts, self.wheels[i])

    def control_loop(self):
        if self.ebrake_active:
            for wheel in self.wheels:
                wheel.reset()
            msg = Int32MultiArray()
            msg.data = [0, 0, 0, 0]
            self.motor_pub.publish(msg)
            return

        # Calculate target velocities for differential drive
        if abs(self.desired_vel) < 0.001 and abs(self.angular_vel) > 0.001:
            # Spot turning
            turning_pwm = self.angular_vel * ANGULAR_SCALE
            left_speed = -turning_pwm / ANGULAR_SCALE
            right_speed = turning_pwm / ANGULAR_SCALE
        else:
            # Normal driving with steering
            base_speed = self.desired_vel * (2.0 if self.turbo_active else 1.0)
            steering_offset = self.angular_vel * ANGULAR_SCALE / ANGULAR_SCALE
            left_speed = base_speed - steering_offset
            right_speed = base_speed + steering_offset

        current_time = time.time()
        
        # Update wheel PWMs using PID control
        self.update_wheel_pwm(self.wheels[0], right_speed, current_time)  # FR
        self.update_wheel_pwm(self.wheels[1], left_speed, current_time)   # FL
        self.update_wheel_pwm(self.wheels[2], right_speed, current_time)  # BR
        self.update_wheel_pwm(self.wheels[3], left_speed, current_time)   # BL

        # Prepare final PWM values
        pwm_values = []
        for wheel in self.wheels:
            final_pwm = -wheel.current_pwm if wheel.invert else wheel.current_pwm
            pwm_values.append(final_pwm)

        # Publish commands
        if len(pwm_values) == 4:
            msg = Int32MultiArray()
            msg.data = [int(pwm) for pwm in pwm_values]
            self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = MotorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Int32MultiArray()
        msg.data = [0, 0, 0, 0]
        controller.motor_pub.publish(msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()