#!/usr/bin/env python3
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node

class TwistToTwistStampedNode(Node):
    def __init__(self):
        super().__init__('vel_stamper_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist = msg
        self.publisher_.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStampedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()