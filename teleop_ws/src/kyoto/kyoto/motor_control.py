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
MAX_PWM = 80  # Normal driving max PWM
MAX_BREAK_PWM = 180  # Maximum PWM for breaking static friction
WHEEL_DIAMETER = 0.127  # meters
WHEEL_CIRCUMFERENCE = np.pi * WHEEL_DIAMETER
ENCODER_CPR = 204.19/2

# Tuning Parameters
STATIC_BREAK_PWM = 255     # PWM to break static friction
STATIC_BREAK_TIME = 0.1    # Time to apply break-free PWM (seconds)
VELOCITY_THRESHOLD = 0.05  # Threshold to consider wheel moving
PWM_RAMP_RATE = 5         # PWM change per control cycle

# Control Parameters
ANGULAR_SCALE = 100.0     # Scale factor for steering while driving
SPOT_TURN_SCALE = 350.0    # Scale factor for spot turning

# PID Gains
KP = 40.0
KI = 80.0
KD = 0.2
INTEGRAL_LIMIT = 25.0

class MotorState(Enum):
    STOPPED = 0
    BREAKING_STATIC = 1
    NORMAL_OPERATION = 2
    STOPPING = 3

class MovingAverageFilter:
    def __init__(self, window_size=2):
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
        
        if (error > 0 and self.last_error < 0) or (error < 0 and self.last_error > 0):
            self.integral = 0
            
        P = self.Kp * error
        
        if abs(error) < 1.0:
            self.integral = np.clip(
                self.integral + error * dt,
                self.integral_limits[0],
                self.integral_limits[1]
            )
        I = self.Ki * self.integral
        
        D = self.Kd * (error - self.last_error) / dt if dt > 0 else 0
        
        self.last_error = error
        self.last_time = current_time
        
        return P + I + D

class WheelController:
    def __init__(self, invert=False):
        self.pid = PIDController()
        self.velocity_filter = MovingAverageFilter(window_size=2)
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
        
        self.wheels = [
            WheelController(invert=True),   # FR - inverted
            WheelController(invert=False),  # FL
            WheelController(invert=True),   # BR - inverted
            WheelController(invert=False)   # BL
        ]
        
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, self.qos)
        self.create_subscription(Float32, 'desired_steering', self.steering_callback, self.qos)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, self.qos)
        
        self.motor_pub = self.create_publisher(Int32MultiArray, 'motor_pwm', self.qos)
        
        self.desired_vel = 0.0
        self.angular_vel = 0.0
        self.ebrake_active = False
        self.turbo_active = False
        
        self.create_timer(0.02, self.control_loop)
        
    def vel_callback(self, msg):
        self.desired_vel = msg.data
        if abs(msg.data) < 0.001:
            for wheel in self.wheels:
                wheel.change_state(MotorState.STOPPED)

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
        is_spot_turn = abs(self.angular_vel) > 0.001 and wheel.state == MotorState.STOPPED
        if is_spot_turn:
            direct_pwm = int(np.clip(target_velocity / 2, -MAX_PWM, MAX_PWM))
            wheel.current_pwm = direct_pwm
            return

        if wheel.state == MotorState.STOPPED:
            if abs(target_velocity) > 0.001:
                wheel.change_state(MotorState.BREAKING_STATIC)
                wheel.current_pwm = np.sign(target_velocity) * MAX_BREAK_PWM
            else:
                wheel.current_pwm = 0
                
        elif wheel.state == MotorState.BREAKING_STATIC:
            if current_time - wheel.state_start_time > STATIC_BREAK_TIME:
                wheel.change_state(MotorState.NORMAL_OPERATION)
                
        elif wheel.state == MotorState.NORMAL_OPERATION:
            if abs(target_velocity) < 0.001:
                wheel.change_state(MotorState.STOPPING)
            else:
                pid_output = wheel.pid.compute(target_velocity, wheel.current_velocity, current_time)
                target_pwm = int(np.clip(pid_output, -MAX_PWM, MAX_PWM))
                
                pwm_diff = target_pwm - wheel.current_pwm
                if abs(pwm_diff) > PWM_RAMP_RATE:
                    wheel.current_pwm += np.sign(pwm_diff) * PWM_RAMP_RATE
                else:
                    wheel.current_pwm = target_pwm
                    
        elif wheel.state == MotorState.STOPPING:
            wheel.current_pwm = 0
            if abs(wheel.current_velocity) < VELOCITY_THRESHOLD:
                wheel.change_state(MotorState.STOPPED)
                
        if wheel.state == MotorState.BREAKING_STATIC:
            wheel.current_pwm = np.clip(wheel.current_pwm, -MAX_BREAK_PWM, MAX_BREAK_PWM)
        else:
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

        current_time = time.time()

        if abs(self.desired_vel) < 0.001 and abs(self.angular_vel) > 0.001:
            turning_speed = self.angular_vel * SPOT_TURN_SCALE
            left_speed = -turning_speed
            right_speed = turning_speed
        else:
            base_speed = self.desired_vel * (2.0 if self.turbo_active else 1.0)
            steering_offset = self.angular_vel * ANGULAR_SCALE
            left_speed = base_speed - steering_offset
            right_speed = base_speed + steering_offset

        self.update_wheel_pwm(self.wheels[0], right_speed, current_time)  # FR
        self.update_wheel_pwm(self.wheels[1], left_speed, current_time)   # FL
        self.update_wheel_pwm(self.wheels[2], right_speed, current_time)  # BR
        self.update_wheel_pwm(self.wheels[3], left_speed, current_time)   # BL

        pwm_values = []
        for wheel in self.wheels:
            final_pwm = -wheel.current_pwm if wheel.invert else wheel.current_pwm
            pwm_values.append(final_pwm)

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