#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys

class PosePublisher(Node):
    def __init__(self, x, y, z):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.pose = PoseWithCovarianceStamped()
        self.pose.pose.pose.position.x = x
        self.pose.pose.pose.position.y = y
        self.pose.pose.pose.position.z = z

    def publish_pose(self):
        self.get_logger().info(f'Publishing: {self.pose}')
        self.publisher_.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)
    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    node = PosePublisher(x, y, z)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()