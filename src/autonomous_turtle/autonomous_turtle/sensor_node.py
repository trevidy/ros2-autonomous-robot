import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Creating the subscription
        self.scan_sub = self.create_subscription(
            LaserScan, # 1) Message type. It's a standard message type containing an array of distances from a LIDAR sensor. 
            '/scan', # 2) Topic name. This is where the sensor data will be published.
            self.scan_callback, # 3) Callback function. Every time a new message arrives on /scan, the ROS will automatically run the function named scan_callback.
            10 # 4) History Depth (QoS). The queue size for incoming messages. It will keep the 10 most recent messages in a buffer before dropping the oldest ones.
        )

        self.latest_scan = None # To store the latest scan data

    def scan_callback(self, msg): # Callback function to handle incoming LaserScan messages
        self.latest_scan = msg # Store the latest scan data

    def main(args=None):
        rclpy.init(args=args)
        node = SensorNode()
        rclpy.spin(node)
        rclpy.shutdown()
    