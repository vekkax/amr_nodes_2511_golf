#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import statistics as sts
import time
from custom_interfaces.msg import TtcDebug  # Assuming this is the custom message for debug

import math
import sys



class TTC(Node):  # Redefine node class
    def __init__(self):
        super().__init__("time_to_collision_node")  # Redefine node name

        self.declare_parameter('scan_topic', '/scan')
        self.scan = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        self.lidar_subs = self.create_subscription(LaserScan, self.scan, self.scan_callback, 10)
        self.cmd_vel_subs = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10)
        
        self.ttc_debug= self.create_publisher(TtcDebug, "/ttc_debug", 10)
        self.aeb_pub = self.create_publisher(Twist, "/cmd_vel_aeb", 10)
        self.aeb_active = self.create_publisher(Bool, "/aeb", 10)
        self.TTC = self.create_publisher(Float32, "/ttc", 10)
        


        self.time_lapse = 0.001
        self.timer = self.create_timer(self.time_lapse, self.timer_callback)

        self.cmd_break= Twist()
        self.cmd_break.linear.x = 0.0
        self.cmd_break.angular.z = 0.0

        self.aeb_data = Bool()
        self.aeb_data.data = False 

        self.vel_x = float()

        self.dist_prev= float()
        self.dist = float()
        
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('rumble', False)
        self.rumble = self.get_parameter('rumble').get_parameter_value().bool_value
        
        if self.rumble:
            self.rumble_pub = self.create_publisher(Bool, "/joy_rumble", 10)

        self.margin=0.5

        self.ttc = 0.0  # Time to collision value
        self.ttc_prev = 0.0  # Previous time to collision value

    def timer_callback(self):         
        dist = float(self.dist)

        dist_d = dist - self.dist_prev
        dist_d = dist_d / self.time_lapse

        if dist_d != 0.0:
            # Calculate time to collision (TTC)
            self.ttc = 100*-dist/dist_d
            

        self.TTC.publish(Float32(data=self.ttc))

        if self.vel_x>0.0 and self.dist < self.margin and not(self.aeb_data.data):            
            

            if 0.0<(self.ttc) <2.0:
               
                # AEB activation
                self.get_logger().warn("🔴 AEB activated")
                self.aeb_data.data=True
                if self.rumble:
                    self.rumble_pub.publish(self.aeb_data)
                self.aeb_active.publish(self.aeb_data)
                self.aeb_pub.publish(self.cmd_break)
                
                

            else:
                
                self.aeb_data.data=False

            self.aeb_active.publish(self.aeb_data)   

        # if AEB is already active
        elif self.aeb_data.data:

            # AEB is active, keep active
            self.get_logger().warn("🔴 AEB is active")

            self.aeb_data.data=True
            if self.rumble:
                self.rumble_pub.publish(self.aeb_data)
            self.cmd_break.linear.x = 5*(self.dist-self.margin)-0.25
            self.aeb_pub.publish(self.cmd_break)

            # if the distance is greater than 0.4, deactivate AEB
            if self.dist > self.margin:
                #DEACTIVATION
                self.get_logger().warn("🟢 AEB deactivated")
                self.cmd_break.linear.x = 0.0
                self.aeb_pub.publish(self.cmd_break)
                self.aeb_data.data=False

            self.aeb_active.publish(self.aeb_data)

        self.dist_prev = dist

        if self.debug:
            # Publish debug information
            ttc_debug_msg = TtcDebug()
            ttc_debug_msg.ttc = self.ttc
            ttc_debug_msg.dist = dist
            ttc_debug_msg.vel_x = self.vel_x
            ttc_debug_msg.dist_d =  dist_d
            ttc_debug_msg.aeb_active = self.aeb_data.data
            self.ttc_debug.publish(ttc_debug_msg)

    def scan_callback(self, data: LaserScan, half_window =2):
        self.dist_prev = self.dist
        self.dist= data.ranges[179]
    
        
    def vel_callback(self, data: Twist):
        self.vel_x=data.linear.x
    

def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = TTC()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
