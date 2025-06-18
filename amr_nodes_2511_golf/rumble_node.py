#!/usr/bin/env python3

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import pygame
import time
# libreria cliente de python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool



class RumbleNode(Node):
    def __init__(self):
        super().__init__("rumble_node")
        pygame.init()
        pygame.joystick.init()

        self.rumble_sub = self.create_subscription(Bool, "/joy_rumble", self.rumble_callback, 10)
        self.rumble_pub = self.create_publisher(Bool, "/joy_rumble", 10)
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().warn("No joystick detected.")
            self.joystick = None
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Detected joystick: {self.joystick.get_name()}")

    def rumble_callback(self,msg:Bool,duration=250, strong=1.0, weak=1.0):
        rumble=msg
        if rumble.data:
            if self.joystick is not None:
                try:
                    self.joystick.rumble(strong, weak, int(duration))
                    rumble.data=False
                    time.sleep(duration/1000)
                    self.joystick.stop_rumble()
                    self.rumble_pub.publish(rumble)
                except AttributeError:
                    self.get_logger().warn("Rumble not supported by this joystick/driver.")

def main(args=None):
    rclpy.init(args=args)

    # Create node object
    node = RumbleNode()

    # Keep the node running
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()