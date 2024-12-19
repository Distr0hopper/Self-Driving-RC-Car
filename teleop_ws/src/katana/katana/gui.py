#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QProgressBar, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor

class StatusLight(QWidget):
    def __init__(self, label, invert_color=False):
        super().__init__()
        self.layout = QHBoxLayout()
        self.layout.setSpacing(5)
        self.layout.setContentsMargins(5, 5, 5, 5)
        
        self.label = QLabel(label)
        self.indicator = QLabel()
        self.indicator.setFixedSize(15, 15)
        self.invert_color = invert_color
        
        self.layout.addWidget(self.label)
        self.layout.addStretch()
        self.layout.addWidget(self.indicator)
        
        self.setLayout(self.layout)
        self.setState(False)
        
    def setState(self, state):
        if self.invert_color:
            color = "red" if state else "green"
        else:
            color = "green" if state else "red"
        self.indicator.setStyleSheet(
            f"background-color: {color}; border-radius: 7px;"
        )

class MotorDisplay(QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        layout = QVBoxLayout()
        
        # PWM bar (-255 to 255)
        self.pwm_bar = QProgressBar()
        self.pwm_bar.setRange(-255, 255)
        self.pwm_bar.setFormat("%v")
        
        # Encoder count
        self.encoder_label = QLabel("Counts: 0")
        
        # Add velocity label
        self.velocity_label = QLabel("Velocity: 0.00 m/s")
        
        layout.addWidget(self.pwm_bar)
        layout.addWidget(self.encoder_label)
        layout.addWidget(self.velocity_label)
        self.setLayout(layout)
    
    def updatePWM(self, pwm):
        self.pwm_bar.setValue(pwm)
        if pwm > 0:
            self.pwm_bar.setStyleSheet("QProgressBar::chunk { background-color: #00ff00; }")
        elif pwm < 0:
            self.pwm_bar.setStyleSheet("QProgressBar::chunk { background-color: #ff0000; }")
        else:
            self.pwm_bar.setStyleSheet("")
            
    def updateEncoder(self, count):
        self.encoder_label.setText(f"Counts: {count}")
        
    def updateVelocity(self, velocity):
        self.velocity_label.setText(f"Velocity: {velocity:.2f} m/s")

class TeleopGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Monitor")
        self.setMinimumSize(800, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Control Inputs Group
        input_group = QGroupBox("Control Inputs")
        input_layout = QVBoxLayout()
        
        # Desired Velocity display (-5 to 5 m/s)
        vel_layout = QHBoxLayout()
        vel_layout.addWidget(QLabel("Desired Velocity:"))
        self.vel_value = QLabel("0.00 m/s")
        self.desired_vel_bar = QProgressBar()
        self.desired_vel_bar.setRange(-5000, 5000)
        vel_layout.addWidget(self.vel_value)
        vel_layout.addWidget(self.desired_vel_bar)
        input_layout.addLayout(vel_layout)
        
        # Manual Velocity display (-5 to 5 m/s)
        manual_vel_layout = QHBoxLayout()
        manual_vel_layout.addWidget(QLabel("Manual Velocity:"))
        self.manual_vel_value = QLabel("0.00 m/s")
        self.manual_vel_bar = QProgressBar()
        self.manual_vel_bar.setRange(-5000, 5000)
        manual_vel_layout.addWidget(self.manual_vel_value)
        manual_vel_layout.addWidget(self.manual_vel_bar)
        input_layout.addLayout(manual_vel_layout)
        
        # Desired Steering display (-0.5 to 0.5 rad/s)
        steering_layout = QHBoxLayout()
        steering_layout.addWidget(QLabel("Desired Steering:"))
        self.steering_value = QLabel("0.00 rad/s")
        self.steering_bar = QProgressBar()
        self.steering_bar.setRange(-500, 500)
        steering_layout.addWidget(self.steering_value)
        steering_layout.addWidget(self.steering_bar)
        input_layout.addLayout(steering_layout)
        
        # Manual Steering display (-0.5 to 0.5 rad/s)
        manual_steering_layout = QHBoxLayout()
        manual_steering_layout.addWidget(QLabel("Manual Steering:"))
        self.manual_steering_value = QLabel("0.00 rad/s")
        self.manual_steering_bar = QProgressBar()
        self.manual_steering_bar.setRange(-500, 500)
        manual_steering_layout.addWidget(self.manual_steering_value)
        manual_steering_layout.addWidget(self.manual_steering_bar)
        input_layout.addLayout(manual_steering_layout)
        
        input_group.setLayout(input_layout)
        main_layout.addWidget(input_group)
        
        # Status Group
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        # Top row with indicators
        indicators_layout = QHBoxLayout()
        indicators_layout.setSpacing(20)
        
        self.ebrake_status = StatusLight("E-Brake", invert_color=True)
        self.turbo_status = StatusLight("Turbo")
        self.controller_status = StatusLight("Controller")
        self.manual_mode_status = StatusLight("Manual Mode")
        self.auto_mode_status = StatusLight("Auto Mode")
        self.return_status = StatusLight("Return")  # Added return status light
        
        indicators_layout.addWidget(self.ebrake_status)
        indicators_layout.addStretch()
        indicators_layout.addWidget(self.turbo_status)
        indicators_layout.addStretch()
        indicators_layout.addWidget(self.controller_status)
        indicators_layout.addStretch()
        indicators_layout.addWidget(self.manual_mode_status)
        indicators_layout.addStretch()
        indicators_layout.addWidget(self.auto_mode_status)
        indicators_layout.addStretch()
        indicators_layout.addWidget(self.return_status)  # Added to layout
        
        status_layout.addLayout(indicators_layout)
        
        # Add car velocity display
        car_vel_layout = QHBoxLayout()
        car_vel_layout.addWidget(QLabel("Car Velocity:"))
        self.car_velocity_label = QLabel("0.00 m/s")
        car_vel_layout.addWidget(self.car_velocity_label)
        car_vel_layout.addStretch()
        status_layout.addLayout(car_vel_layout)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # Motors Group
        motors_group = QGroupBox("Motors")
        motors_layout = QHBoxLayout()
        
        self.motors = {
            'FR': MotorDisplay("Front Right"),
            'FL': MotorDisplay("Front Left"),
            'BR': MotorDisplay("Back Right"),
            'BL': MotorDisplay("Back Left")
        }
        
        for motor in self.motors.values():
            motors_layout.addWidget(motor)
            
        motors_group.setLayout(motors_layout)
        main_layout.addWidget(motors_group)
        
        # Limits Group
        limits_group = QGroupBox("Limits")
        limits_layout = QHBoxLayout()
        self.max_vel_label = QLabel("Max Velocity: 0.00 m/s")
        self.max_steering_label = QLabel("Max Steering: 0.00 rad/s")
        limits_layout.addWidget(self.max_vel_label)
        limits_layout.addWidget(self.max_steering_label)
        limits_group.setLayout(limits_layout)
        main_layout.addWidget(limits_group)
        
        self.show()

class TeleopMonitor(Node):
    def __init__(self):
        super().__init__('teleop_monitor')
        
        self.app = QApplication(sys.argv)
        self.gui = TeleopGUI()
        
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Add new subscriptions for velocities
        self.create_subscription(Float32, '/car/velocity', self.car_velocity_callback, self.qos)
        self.create_subscription(Float32MultiArray, '/wheel_velocities', self.wheel_velocities_callback, self.qos)
        
        # Original control subscriptions
        self.create_subscription(Float32, 'desired_velocity', self.vel_callback, self.qos)
        self.create_subscription(Float32, 'desired_steering', self.steering_callback, self.qos)
        
        # Manual control subscriptions
        self.create_subscription(Float32, 'manual_velocity', self.manual_vel_callback, self.qos)
        self.create_subscription(Float32, 'manual_steering', self.manual_steering_callback, self.qos)
        
        # Other subscriptions
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, self.qos)
        self.create_subscription(Float32, 'max_vel', self.max_vel_callback, self.qos)
        self.create_subscription(Float32, 'max_steering', self.max_steering_callback, self.qos)
        self.create_subscription(Bool, 'heartbeat', self.heartbeat_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'motor_pwm', self.pwm_callback, self.qos)
        self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, self.qos)
        
        # Control mode subscriptions
        self.create_subscription(Bool, 'manual_mode', self.manual_mode_callback, self.qos)
        self.create_subscription(Bool, 'auto_mode', self.auto_mode_callback, self.qos)
        self.create_subscription(Bool, 'return', self.return_callback, self.qos)  # Added return subscription
        
        self.create_timer(0.02, self.update_gui)
        
    # Added return callback
    def return_callback(self, msg):
        self.gui.return_status.setState(msg.data)
        
    # Manual control callbacks
    def manual_vel_callback(self, msg):
        scaled_value = int(msg.data * 1000)
        self.gui.manual_vel_bar.setValue(scaled_value)
        self.gui.manual_vel_value.setText(f"{msg.data:.2f} m/s")
        
    def manual_steering_callback(self, msg):
        scaled_value = int(msg.data * 1000)
        self.gui.manual_steering_bar.setValue(scaled_value)
        self.gui.manual_steering_value.setText(f"{msg.data:.2f} rad/s")
        
    # Control mode callbacks
    def manual_mode_callback(self, msg):
        self.gui.manual_mode_status.setState(msg.data)
        
    def auto_mode_callback(self, msg):
        self.gui.auto_mode_status.setState(msg.data)
        
    # Existing callbacks
    def car_velocity_callback(self, msg):
        self.gui.car_velocity_label.setText(f"{msg.data:.2f} m/s")
        
    def wheel_velocities_callback(self, msg):
        motors = ['FR', 'FL', 'BR', 'BL']
        for i, motor in enumerate(motors):
            if i < len(msg.data):
                self.gui.motors[motor].updateVelocity(msg.data[i])
        
    def vel_callback(self, msg):
        scaled_value = int(msg.data * 1000)
        self.gui.desired_vel_bar.setValue(scaled_value)
        self.gui.vel_value.setText(f"{msg.data:.2f} m/s")
        
    def steering_callback(self, msg):
        scaled_value = int(msg.data * 1000)
        self.gui.steering_bar.setValue(scaled_value)
        self.gui.steering_value.setText(f"{msg.data:.2f} rad/s")
        
    def ebrake_callback(self, msg):
        self.gui.ebrake_status.setState(msg.data)
        
    def turbo_callback(self, msg):
        self.gui.turbo_status.setState(msg.data)
        
    def max_vel_callback(self, msg):
        self.gui.max_vel_label.setText(f"Max Velocity: {msg.data:.2f} m/s")
        
    def max_steering_callback(self, msg):
        self.gui.max_steering_label.setText(f"Max Steering: {msg.data:.2f} rad/s")
        
    def heartbeat_callback(self, msg):
        self.gui.controller_status.setState(msg.data)
        
    def pwm_callback(self, msg):
        motors = ['FR', 'FL', 'BR', 'BL']
        for i, motor in enumerate(motors):
            if i < len(msg.data):
                self.gui.motors[motor].updatePWM(msg.data[i])
                
    def encoder_callback(self, msg):
        motors = ['FR', 'FL', 'BR', 'BL']
        for i, motor in enumerate(motors):
            if i < len(msg.data):
                self.gui.motors[motor].updateEncoder(msg.data[i])
        
    def update_gui(self):
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMonitor()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not node.gui.isVisible():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()