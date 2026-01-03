import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.target_pub = self.create_publisher(
            Float32,
            '/target_yaw',
            10
        )

        self.state = 'FORWARD'
        self.front_distance = float('inf')
        self.obstacle_threshold = 0.6
        self.current_yaw = 0.0

    def scan_callback(self, msg):

        # Use the front 10 degrees of LIDAR
        center = len(msg.ranges) // 2
        self.front_distance = min(msg.ranges[center-5:center+5])

        # State machine logic
        if self.state == 'FORWARD' and self.front_distance < self.obstacle_threshold:
            self.state = 'TURNING' # Switch to TURNING state
            target = Float32() # create target yaw message
            target.data = math.pi / 2 # sets the target yaw to 90 degrees
            self.target_pub.publish(target) # publish target yaw
            self.get_logger().info("Obstacle detected, switching to TURNING state.")
        
        elif self.state == 'TURNING' and self.front_distance > 1.0:
            self.state = 'FORWARD'