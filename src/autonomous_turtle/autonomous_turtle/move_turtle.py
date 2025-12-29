## Imports
import rclpy # ROS 2 Python client library
from rclpy.node import Node # Base class for ROS 2 nodes
from geometry_msgs.msg import Twist # Message type for velocity commands. Includes linear(straight line) and angular(turning) components.

class MoveTurtle(Node): # Node class to move the turtle autonomously
    def __init__(self):
        super().__init__('move_turtle') # Initialize the node with the name 'move_turtle'
        
        # Publisher to control the turtle's velocity
        self.publisher_ = self.create_publisher( # Create a publisher to send velocity commands
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Timer: run callback every 0.1 seconds
        self.timer = self.create_timer(0.1, self.move_callback) # Create a timer that triggers the move_callback every 0.1 seconds for continuous movement
        self.get_logger().info("Autonomous node has started.") # Log message indicating the node has started

    def move_callback(self): # Callback function to publish velocity commands
        # Every time the timer triggers, publish a Twist message to move the turtle

        msg = Twist()
        msg.linear.x = 2.0  # Moves forward at 2.0 m/s
        msg.angular.z = 1.0  # Rotate left at 1.0 rad/s
        self.publisher_.publish(msg)


def main(args=None):
        rclpy.init(args=args) # Starts ROS 2 communication
        node = MoveTurtle() # Create an instance of the MoveTurtle node
        rclpy.spin(node) # Keep the node running to listen for events and execute callbacks
        node.destroy_node() # Clean up and destroy the node when done
        rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()