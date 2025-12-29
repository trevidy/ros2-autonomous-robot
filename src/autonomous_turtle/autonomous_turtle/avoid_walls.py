import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from enum import Enum

class State(Enum): # Introduces a state machine for wall avoidance
    FORWARD = 0 
    TURNING = 1


class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')

        self.publisher_ = self.create_publisher( # Publisher for velocity commands
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.subscription = self.create_subscription( # Subscription to turtle's pose
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.pose = None # To store the current pose of the turtle
        
        self.state = State.FORWARD # Initial state
        self.turn_start_time = None # To track when the turtle started turning
        
        self.timer = self.create_timer(0.1,self.control_loop) # Timer for control loop
        self.get_logger().info("Wall Avoider node has started.")

    def pose_callback(self, msg): # Callback to update the turtle's pose
        self.pose = msg

    def control_loop(self): # Control loop to avoid walls
        if self.pose is None:
            return

        msg = Twist() # Create a Twist message for velocity commands
        margin = 1.0 # Distance from wall to start avoiding
        
        near_wall = ( # Check if the turtle is near any wall
            self.pose.x < margin or 
            self.pose.x > 11.0 - margin or
            self.pose.y < margin or
            self.pose.y > 11.0 - margin
        )
        
        now = self.get_clock().now().nanoseconds / 1e9 # Current time in seconds

        if self.state == State.FORWARD:
            if near_wall: 
                self.state = State.TURNING # Switch to TURNING state if near a wall
                self.turn_start_time = now # Record the time when turning starts
            msg.linear.x = 2.0 # Move forward
            msg.angular.z = 0.0 # No rotation
        
        elif self.state == State.TURNING:
            msg.linear.x = 1.5 # Slow forward movement while turning
            msg.angular.z = 2.5 # Turn rate

            if now - self.turn_start_time > 1.2:
                self.state = State.FORWARD # Switch back to FORWARD state after turning

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()