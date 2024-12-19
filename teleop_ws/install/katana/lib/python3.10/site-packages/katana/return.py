#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import time

class ReturnDockingController(Node):
    def __init__(self):
        super().__init__('return_docking_controller')
        
        # Image dimensions and zones
        self.IMAGE_WIDTH = 1920
        # 40% - 20% - 40% split
        self.LEFT_THRESH = self.IMAGE_WIDTH * 0.4     # 768px
        self.RIGHT_THRESH = self.IMAGE_WIDTH * 0.6    # 1152px
        
        # Distance thresholds
        self.SLOW_APPROACH_DISTANCE = 1.5  # meters to start slow approach
        self.DOCKING_DISTANCE = 0.3        # meters considered as "docked"
        
        # Speeds for different phases
        self.SEARCH_ROTATION_SPEED = 0.3   # rad/s for searching
        self.FORWARD_SPEED = 0.3           # m/s for normal approach
        self.SLOW_SPEED = 0.2              # m/s for close approach
        self.TURNING_SPEED = 0.2           # rad/s for centering
        
        # Time windows for actions
        self.SEARCH_ROTATE_WINDOW = 1.50   # seconds to rotate during search
        self.SEARCH_PAUSE_WINDOW = 2.0     # seconds to pause during search
        self.ALIGNMENT_TURN_WINDOW = 0.2   # seconds for alignment turns
        self.ALIGNMENT_STEADY_WINDOW = 1.0 # seconds to pause between alignment turns
        self.FORWARD_WINDOW = 1.0          # seconds to move forward
        self.SLOW_FORWARD_WINDOW = 0.5     # seconds to move forward when close
        self.BACKUP_WINDOW = 2.0           # seconds to backup
        
        # Search state tracking
        self.search_is_rotating = True  # Start with rotation
        self.alignment_is_turning = True  # Start with turning in alignment
        
        # QoS profile
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers - Changed to return_velocity and return_steering
        self.return_vel_pub = self.create_publisher(Float32, 'return_velocity', self.qos)
        self.return_steering_pub = self.create_publisher(Float32, 'return_steering', self.qos)
        
        # Subscribers
        self.create_subscription(Bool, 'ebrake', self.ebrake_callback, self.qos)
        self.create_subscription(Bool, 'manual_mode', self.manual_mode_callback, self.qos)
        self.create_subscription(Bool, 'return', self.return_callback, self.qos)  # Changed from auto_mode to return
        self.create_subscription(Int32, 'tag_id', self.tag_id_callback, self.qos)
        self.create_subscription(Float32MultiArray, 'tag_data', self.tag_data_callback, self.qos)
        
        # State variables
        self.ebrake_active = False
        self.manual_mode = False
        self.return_active = False  # Changed from auto_mode
        self.current_tag_id = None
        self.tag_center_x = None
        self.tag_distance = None
        self.last_tag_time = None
        
        # Action window tracking
        self.action_start_time = None
        self.current_action = None
        
        # State machine states - Simplified for single tag return
        self.SEARCHING = 'searching'
        self.ALIGNING = 'aligning'
        self.APPROACHING = 'approaching'
        self.SLOW_APPROACH = 'slow_approach'
        self.FINISHED = 'finished'
        
        self.current_state = self.SEARCHING
        self.get_logger().info('State: Initialized')
        
        # Create timer for checking state transitions and maintaining commands
        self.create_timer(0.1, self.check_state_transition)  # 10Hz for smoother commands
        
    def start_action(self, action_name):
        """Start a new timed action"""
        self.action_start_time = time.time()
        self.current_action = action_name
        self.get_logger().info(f'Starting action: {action_name}')
        
    def action_completed(self):
        """Check if current action window has completed"""
        if self.action_start_time is None:
            return True
            
        elapsed = time.time() - self.action_start_time
        
        if self.current_action == 'search_rotate':
            return elapsed >= self.SEARCH_ROTATE_WINDOW
        elif self.current_action == 'search_pause':
            return elapsed >= self.SEARCH_PAUSE_WINDOW
        elif self.current_action == 'align_turn':
            return elapsed >= self.ALIGNMENT_TURN_WINDOW
        elif self.current_action == 'align_steady':
            return elapsed >= self.ALIGNMENT_STEADY_WINDOW
        elif self.current_action == 'forward':
            return elapsed >= self.FORWARD_WINDOW
        elif self.current_action == 'slow_forward':
            return elapsed >= self.SLOW_FORWARD_WINDOW
        return True
        
    def handle_search(self):
        """Handle search pattern rotation and pausing"""
        if self.action_completed():
            if self.search_is_rotating:
                # Switch to pause
                self.search_is_rotating = False
                self.start_action('search_pause')
                self.get_logger().info('Search: Starting pause')
            else:
                # Switch to rotation
                self.search_is_rotating = True
                self.start_action('search_rotate')
                self.get_logger().info('Search: Starting rotation')
                
        # Maintain commands during the entire window
        if self.search_is_rotating:
            self.publish_commands(0.0, self.SEARCH_ROTATION_SPEED)
        else:
            self.publish_commands(0.0, 0.0)
            
    def handle_alignment(self, turn_dir):
        """Handle alignment with steady pauses between turns"""
        if self.action_completed():
            if self.alignment_is_turning:
                # Switch to steady pause
                self.alignment_is_turning = False
                self.start_action('align_steady')
                self.get_logger().info('Alignment: Starting steady pause')
            else:
                # Switch back to turning
                self.alignment_is_turning = True
                self.start_action('align_turn')
                self.get_logger().info('Alignment: Starting turn')
        
        # Maintain commands based on current alignment phase
        if self.alignment_is_turning:
            if turn_dir == 'right':
                self.publish_commands(0.0, self.TURNING_SPEED)
            elif turn_dir == 'left':
                self.publish_commands(0.0, -self.TURNING_SPEED)
        else:
            self.publish_commands(0.0, 0.0)  # Steady pause
        
    def publish_commands(self, velocity, steering):
        vel_msg = Float32()
        vel_msg.data = velocity
        self.return_vel_pub.publish(vel_msg)
        
        steering_msg = Float32()
        steering_msg.data = steering
        self.return_steering_pub.publish(steering_msg)
        
    def tag_lost(self):
        return (self.last_tag_time is None or 
                time.time() - self.last_tag_time > 0.5)
                
    def determine_turn_direction(self):
        """Determine which way to turn based on 40-20-40 width rule"""
        if self.tag_center_x is None:
            return 'right'  # Default turn direction
            
        if self.tag_center_x < self.LEFT_THRESH:  # In left 40%
            return 'left'   # Turn LEFT if tag is in left zone
        elif self.tag_center_x > self.RIGHT_THRESH:  # In right 40%
            return 'right'  # Turn RIGHT if tag is in right zone
        else:  # In center 20%
            return 'center' # Go straight if tag is in center zone
            
    def is_centered(self):
        """Check if tag is in center 20% of image"""
        if self.tag_center_x is None:
            return False
        return self.LEFT_THRESH <= self.tag_center_x <= self.RIGHT_THRESH
            
    def check_state_transition(self):
        """Check if we should transition states and maintain commands"""
        if not self.return_active or self.manual_mode or self.ebrake_active:
            self.publish_commands(0.0, 0.0)
            return
            
        # State machine logic
        if self.current_state == self.SEARCHING:
            if self.current_tag_id == 1 and not self.tag_lost():  # Looking for tag ID 1
                self.set_state(self.ALIGNING)
                self.alignment_is_turning = True
                self.start_action('align_turn')
            else:
                self.handle_search()
                
        elif self.current_state == self.ALIGNING:
            if self.current_tag_id != 1 or self.tag_lost():
                self.set_state(self.SEARCHING)
                self.search_is_rotating = True
                self.start_action('search_rotate')
            else:
                turn_dir = self.determine_turn_direction()
                if turn_dir == 'center':  # Tag is in center 20%
                    if self.tag_distance <= self.SLOW_APPROACH_DISTANCE:
                        self.set_state(self.SLOW_APPROACH)
                        self.start_action('slow_forward')
                    else:
                        self.set_state(self.APPROACHING)
                        self.start_action('forward')
                else:
                    self.handle_alignment(turn_dir)
                    
        elif self.current_state == self.APPROACHING:
            if self.current_tag_id != 1 or self.tag_lost():
                self.set_state(self.SEARCHING)
                self.search_is_rotating = True
                self.start_action('search_rotate')
            elif self.tag_distance is not None:
                if self.tag_distance <= self.SLOW_APPROACH_DISTANCE:
                    self.set_state(self.SLOW_APPROACH)
                    self.start_action('slow_forward')
                else:
                    if self.action_completed():
                        if not self.is_centered():
                            self.set_state(self.ALIGNING)
                            self.alignment_is_turning = True
                            self.start_action('align_turn')
                        else:
                            self.start_action('forward')
                    
                    if self.current_action == 'forward':
                        self.publish_commands(self.FORWARD_SPEED, 0.0)
                        
        elif self.current_state == self.SLOW_APPROACH:
            if self.current_tag_id != 1 or self.tag_lost():
                self.set_state(self.SEARCHING)
                self.search_is_rotating = True
                self.start_action('search_rotate')
            elif self.tag_distance is not None and self.tag_distance <= self.DOCKING_DISTANCE:
                self.set_state(self.FINISHED)
                self.return_active = False
            else:
                if self.action_completed():
                    if not self.is_centered():
                        self.set_state(self.ALIGNING)
                        self.alignment_is_turning = True
                        self.start_action('align_turn')
                    else:
                        self.start_action('slow_forward')
                
                if self.current_action == 'slow_forward':
                    self.publish_commands(self.SLOW_SPEED, 0.0)
                    
        elif self.current_state == self.FINISHED:
            self.publish_commands(0.0, 0.0)
            
    def set_state(self, new_state):
        if new_state != self.current_state:
            self.current_state = new_state
            self.get_logger().info(f'State: {new_state}')
        
    def ebrake_callback(self, msg):
        self.ebrake_active = msg.data
        
    def manual_mode_callback(self, msg):
        self.manual_mode = msg.data
        if msg.data:
            self.return_active = False
            self.set_state(self.SEARCHING)
            self.search_is_rotating = True
            
    def return_callback(self, msg):
        if msg.data and not self.return_active:
            self.return_active = True
            self.manual_mode = False
            self.set_state(self.SEARCHING)
            self.search_is_rotating = True
        elif not msg.data:
            self.return_active = False
            
    def tag_id_callback(self, msg):
        self.current_tag_id = msg.data
        self.last_tag_time = time.time()
        
    def tag_data_callback(self, msg):
        if len(msg.data) >= 11:
            self.tag_center_x = msg.data[8]
            self.tag_distance = msg.data[10]

def main(args=None):
    rclpy.init(args=args)
    node = ReturnDockingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()