import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from turtlebot4_msgs.msg import UserButton

class TurtleBot4FirstNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        # Subscribe to the /hmi/buttons topic (TurtleBot4 Standard HMI buttons)
        self.buttons_subscriber_ = self.create_subscription(
            UserButton,
            '/hmi/buttons',
            self.buttons_callback,
            qos_profile_sensor_data)
        
    def buttons_callback(self, buttons_msg):
        # Button 1 is pressed (index 0)
        if buttons_msg.button[0]:
            self.get_logger().info('Button 1 is pressed!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()