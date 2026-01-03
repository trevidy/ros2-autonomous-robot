#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.safe_distance = 0.6  # meters

        # ROS interfaces
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info("Obstacle avoidance node started")

    def scan_callback(self, scan: LaserScan):
        ranges = scan.ranges

        # Focus on front ±30 degrees
        front_ranges = []

        for i, distance in enumerate(ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if -math.radians(30) < angle < math.radians(30):
                if not math.isinf(distance):
                    front_ranges.append(distance)

        cmd = Twist()

        if front_ranges and min(front_ranges) < self.safe_distance:
            # Obstacle detected → turn
            cmd.angular.z = 0.6
            cmd.linear.x = 0.0
        else:
            # Path is clear → move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()