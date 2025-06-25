#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32

import statistics as sts
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class WallFollowing(Node):  # Redefine node class
    def __init__(self):
        super().__init__("wall_following_node")  # Redefine node name

        #define subscriber and publishers 
        self.declare_parameter('scan_topic', '/scan')
        self.scan = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        self.scan_sub = self.create_subscription(LaserScan, self.scan, self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)  
        

        self.distance=0
        self.time_lapse = 0.01
        self.timer = self.create_timer(self.time_lapse, self.timer_callback)

        self.Trajd = 1.0
        self.CD = float()
        self.AB = float()

        
        self.error_r=0.0
        self.error_r_prev=0.0

        self.error_l=0.0
        self.error_l_prev=0.0

        self.nav_vel=Twist()
        self.nav_vel.linear.x=2.0

        self.laser=LaserScan()
        self.laser.ranges = [5.0] * 360
        self.vel=Twist()

        self.switch_flag = True

        # Plotting variables
        self.declare_parameter('plot', False)
        self.plot_enabled = self.get_parameter('plot').get_parameter_value().bool_value

        if self.plot_enabled:
            self.erro_r = []
            self.erro_l = []
            self.time = []
            self.start_time = self.get_clock().now().nanoseconds * 1e-9
            self.plot_thread = threading.Thread(target=self.start_plotting)
            self.plot_thread.daemon = True
            self.plot_thread.start()

    def scan_callback(self, data:LaserScan):
        self.laser=data

    def vel_callback(self, data:Twist):
        self.vel=data
        

    def get_range(self, data:LaserScan, theta, AC, index):

        a = data.ranges[index] # Cambiar por -90 o 270 si se quiere seguir la pared derecha 
        b = data.ranges[index + theta]

        alpha = math.atan(
            (a * math.cos(math.radians(theta)) - b)
            / (a * math.sin(math.radians(theta)))
        )

        AB = b * math.cos(alpha)
        CD = AB + AC * math.sin(alpha)
        return CD,alpha

    def timer_callback(self):  
        vel=self.vel.linear.x
        distance=self.time_lapse*vel

        left_dist, left_alpha=self.get_range(self.laser, -80, distance,269)
        right_dist, right_alpha=self.get_range(self.laser, 80, distance,89)

        kp=20
        kd=10

        y_r=self.Trajd-right_dist
        self.error_r=-(y_r+distance*math.sin(right_alpha))
        self.error_rd=(self.error_r-self.error_r_prev)/self.time_lapse
        
        y_l=self.Trajd-left_dist
        self.error_l=(y_l+distance*math.sin(left_alpha))
        self.error_ld=(self.error_l-self.error_l_prev)/self.time_lapse
        
        
        if self.switch_flag:
            self.nav_vel.angular.z=-self.error_r*kp - kd*self.error_rd
            self.nav_vel.angular.z=self.limit(self.nav_vel.angular.z,3.0)
            self.get_logger().info("Following Right")
            if self.error_rd<-100:
                self.switch_flag=False
        else:
            self.nav_vel.angular.z=-self.error_l*kp - kd*self.error_ld
            self.nav_vel.angular.z=self.limit(self.nav_vel.angular.z,3.0)
            self.get_logger().info("Following Left")
            if self.error_ld>100:
                self.switch_flag=True

        

        self.cmd_vel_pub.publish(self.nav_vel)

        self.error_r_prev=self.error_r
        self.error_l_prev=self.error_l

        if self.plot_enabled:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.time.append(now - self.start_time)
            self.erro_r.append(self.error_rd)
            self.erro_l.append(self.error_ld)


        


    def limit (self, data, limit:float):
        if data >= limit:
            return limit
        elif data <= -limit:
            return -limit
        else:
            return data
        
    def start_plotting(self):
        self.fig, self.ax = plt.subplots()
        self.line_r, = self.ax.plot([], [], label='erro_r')
        self.line_l, = self.ax.plot([], [], label='erro_l')
        self.ax.legend()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()
        
    def update_plot(self, frame):
        self.line_r.set_data(self.time, self.erro_r)
        self.line_l.set_data(self.time, self.erro_l)
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line_r, self.line_l

def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = WallFollowing()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
