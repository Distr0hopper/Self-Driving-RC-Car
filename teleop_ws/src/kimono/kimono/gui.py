#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QProgressBar)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor

class StatusLight(QWidget):
    def __init__(self, label):
        super().__init__()
        self.layout = QHBoxLayout()
        self.label = QLabel(label)
        self.indicator = QLabel()
        self.indicator.setFixedSize(20, 20)
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.indicator)
        self.setLayout(self.layout)
        self.setState(False)
        
    def setState(self, state):
        color = "green" if state else "red"
        self.indicator.setStyleSheet(
            f"background-color: {color}; border-radius: 10px;"
        )

class TeleopGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Teleop Control Monitor")
        self.setMinimumSize(400, 300)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Velocity display
        vel_layout = QHBoxLayout()
        vel_layout.addWidget(QLabel("Desired Velocity:"))
        self.desired_vel_bar = QProgressBar()
        self.desired_vel_bar.setRange(-100, 100)
        vel_layout.addWidget(self.desired_vel_bar)
        layout.addLayout(vel_layout)
        
        # Steering display
        steering_layout = QHBoxLayout()
        steering_layout.addWidget(QLabel("Steering:"))
        self.steering_bar = QProgressBar()
        self.steering_bar.setRange(-100, 100)
        steering_layout.addWidget(self.steering_bar)
        layout.addLayout(steering_layout)
        
        # Max values display
        max_layout = QHBoxLayout()
        self.max_vel_label = QLabel("Max Velocity: 0.0")
        self.max_steering_label = QLabel("Max Steering: 0.0")
        max_layout.addWidget(self.max_vel_label)
        max_layout.addWidget(self.max_steering_label)
        layout.addLayout(max_layout)
        
        # Status indicators
        self.ebrake_status = StatusLight("E-Brake")
        self.turbo_status = StatusLight("Turbo")
        self.controller_status = StatusLight("Controller")
        
        status_layout = QVBoxLayout()
        status_layout.addWidget(self.ebrake_status)
        status_layout.addWidget(self.turbo_status)
        status_layout.addWidget(self.controller_status)
        layout.addLayout(status_layout)
        
        self.show()

class TeleopMonitor(Node):
    def __init__(self):
        super().__init__('teleop_monitor')
        
        # Create GUI
        self.app = QApplication(sys.argv)
        self.gui = TeleopGUI()
        
        # Subscribe to all topics
        self.create_subscription(Float32, 'desired_vel', self.vel_callback, 10)
        self.create_subscription(Float32, 'desired_steering', self.steering_callback, 10)
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, 10)
        self.create_subscription(Bool, 'turbo', self.turbo_callback, 10)
        self.create_subscription(Float32, 'max_vel', self.max_vel_callback, 10)
        self.create_subscription(Float32, 'max_steering', self.max_steering_callback, 10)
        self.create_subscription(Bool, 'heartbeat', self.heartbeat_callback, 10)
        
        # Timer for GUI updates
        self.create_timer(0.1, self.update_gui)
        
    def vel_callback(self, msg):
        self.gui.desired_vel_bar.setValue(int(msg.data))
        
    def steering_callback(self, msg):
        self.gui.steering_bar.setValue(int(msg.data))
        
    def ebrake_callback(self, msg):
        self.gui.ebrake_status.setState(msg.data)
        
    def turbo_callback(self, msg):
        self.gui.turbo_status.setState(msg.data)
        
    def max_vel_callback(self, msg):
        self.gui.max_vel_label.setText(f"Max Velocity: {msg.data:.1f}")
        
    def max_steering_callback(self, msg):
        self.gui.max_steering_label.setText(f"Max Steering: {msg.data:.1f}")
        
    def heartbeat_callback(self, msg):
        self.gui.controller_status.setState(msg.data)
        
    def update_gui(self):
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMonitor()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.gui.isVisible():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()