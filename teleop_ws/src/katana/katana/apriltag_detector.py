#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TagVisualizer(Node):
    def __init__(self):
        super().__init__('tag_visualizer')
        
        # Create QoS profile for better performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/rs/camera/color/image_raw',
            self.image_callback,
            qos_profile)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            qos_profile)
        self.depth_sub = self.create_subscription(
            Image,
            '/rs/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile)
            
        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/tag_detections_image',
            10)
            
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None
        self.latest_depth = None
        
        self.get_logger().info('Tag Visualizer Initialized')

    def image_callback(self, msg):
        self.latest_image = msg
        self.process_and_publish()

    def detection_callback(self, msg):
        self.latest_detections = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def get_depth_at_point(self, x, y):
        if self.latest_depth is None:
            return None
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="passthrough")
            # Get average depth in a small region around the point
            x, y = int(x), int(y)
            roi = depth_image[max(0, y-2):min(depth_image.shape[0], y+3),
                            max(0, x-2):min(depth_image.shape[1], x+3)]
            depth = np.median(roi[roi > 0]) / 1000.0 if roi.size > 0 else 0
            return depth if depth > 0 else None
        except Exception as e:
            self.get_logger().error(f'Depth error: {str(e)}')
            return None

    def process_and_publish(self):
        if self.latest_image is None or self.latest_detections is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            for detection in self.latest_detections.detections:
                # Draw corners and bounding box
                corners = np.array([(int(corner.x), int(corner.y)) for corner in detection.corners])
                for i in range(4):
                    cv2.line(cv_image, 
                            tuple(corners[i]), 
                            tuple(corners[(i+1)%4]), 
                            (0, 255, 0), 2)
                
                # Center point
                center_x = int(detection.centre.x)
                center_y = int(detection.centre.y)
                cv2.circle(cv_image, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Get depth
                depth = self.get_depth_at_point(center_x, center_y)
                depth_str = f"{depth:.3f}m" if depth is not None else "no depth"
                
                # Display info
                cv2.putText(cv_image, 
                           f"ID:{detection.id} x:{center_x} y:{center_y} z:{depth_str}",
                           (corners[0][0], corners[0][1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, 
                           (0, 255, 0), 
                           2)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    visualizer = TagVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()