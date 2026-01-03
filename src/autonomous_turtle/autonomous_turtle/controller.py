import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        # Publishers / Subscribers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Control Loop Timer (10Hz)
        self.timer = self.create_timer(
            0.1,
            self.control_loop)

        # Robot state
        self.state = 'FORWARD'

        # Control parameters
        self.forward_speed = 0.2
        self.turn_speed_max = 1.0
        self.Kp = 1.5   # Proportional gain for PID controller

        # Obstacle threshold
        self.obstacle_distance = 0.6

        # Turning control
        self.target_yaw = None # Target yaw angle when turning
        self.current_yaw = 0.0 
        self.turn_start_time = None 

        # Sensor data
        self.front_distance = float('inf')
        self.get_logger().info("Autonomous Controller node has started.")

    # ---------------------------------
    # SENSOR CALLBACK
    # ---------------------------------
    def scan_callback(self, msg):
        # Use the front 10 degrees of LIDAR
        center = len(msg.ranges) // 2 # Center index of LIDAR ranges
        window = msg.ranges[center - 5:center + 5] # 10 degree window
        self.front_distance = min(window) # Minimum distance in front

    # ---------------------------------
    # CONTROL LOOP
    # ---------------------------------
    def control_loop(self):
        cmd = Twist()

        if self.state == 'FORWARD':
            if self.front_distance < self.obstacle_distance:
                self.start_turn()
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
        
        elif self.state == 'TURNING':
            error = self.target_yaw - self.current_yaw
            error = self.normalize_angle(error)
            angular_z = self.Kp * error

            # Saturation
            angular_z = max( # Limit angular velocity
                -self.turn_speed_max,
                min(self.turn_speed_max, angular_z)
            )
            cmd.angular.z = angular_z

            # Stop turning if error is small
            if abs(error) < 0.05:
                self.state = 'FORWARD'
                self.get_logger().info("Completed turn, resuming FORWARD state.")
        
        self.cmd_pub.publish(cmd)

    # ---------------------------------
    # TURN INITIALIZATION
    # ---------------------------------
    def start_turn(self):
        self.state = 'TURNING'
        self.target_yaw = self.current_yaw + math.pi / 2
        self.target_yaw = self.normalize_angle(self.target_yaw)
        self.get_logger().info("Obstacle detected! Initiating TURNING state.")

    # ---------------------------------
    # UTILS
    # ---------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
def main(args = None):
    rclpy.init(args=args)
    node = AutonomousController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    

