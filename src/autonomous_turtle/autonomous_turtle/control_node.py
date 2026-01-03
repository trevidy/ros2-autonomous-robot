import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.target_sub = self.create_subscription(
            Float32,
            '/target_yaw',
            self.target_callback,
            10
        )

        self.timer = self.create_timer(0.1,self.control_loop)

        self.target_yaw = 0.0
        self.current_yaw = 0.0
        self.Kp = 1.5

    def target_callback(self, msg):
        self.target_yaw = msg.data

    def control_loop(self):
        error = self.target_yaw - self.current_yaw
        angular_z = self.Kp * error

        cmd = Twist()
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()