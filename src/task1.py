#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class Circle:

    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        node_name = "Task1"

        self.startup = True 
        self.got_start_data = False
        self.got_all_data = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True

    def print_odom_readings(self):
        print(f"x={self.x-self.x0:.2f} m, y={self.y-self.y0:.2f} m yaw={self.theta_z-self.theta_z0:.2f} degrees")

    def main_loop(self):
        status = ""
        round = 0
        while not self.ctrl_c:
            # specify the radius of the circle:
            # if(self.theta_z-self.theta_z0 >= 0.00 and self.theta_z-self.theta_z0 < 0.1):
            #     round = 0
            #     print("round 0")
            if(round == 0):
                #print("round 0")

                path_rad = 0.5 # m
                # linear velocity must be below 0.26m/s:
                lin_vel = 0.2 # m/s
                

                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = lin_vel / path_rad # rad/s

                #self.vel_cmd.linear.x = 0.0
                #self.vel_cmd.angular.z = 0.0
                self.print_odom_readings()
                self.pub.publish(self.vel_cmd)
                self.rate.sleep()
                if(self.theta_z-self.theta_z0 >= -0.1 and self.theta_z-self.theta_z0 < -0.05):
                    round = 1
                    #print("test")
            if(round == 1):
                #print("round 1")
                path_rad = -0.5 # m
                # linear velocity must be below 0.26m/s:
                lin_vel = 0.2 # m/s
                

                self.vel_cmd.linear.x = lin_vel
                self.vel_cmd.angular.z = lin_vel / path_rad # rad/s

                #self.vel_cmd.linear.x = 0.0
                #self.vel_cmd.angular.z = 0.0
                self.print_odom_readings()
                self.pub.publish(self.vel_cmd)
                self.rate.sleep()
                if(self.theta_z-self.theta_z0 >= -0.01 and self.theta_z-self.theta_z0 < 0.1):
                    round = 2
            if(round == 2):
                self.vel_cmd.linear.x = 0.0 # m/s
                self.vel_cmd.angular.z = 0.0 # rad/s

                #print("stopping the robot")

                # publish to the /cmd_vel topic to make the robot stop
                self.pub.publish(self.vel_cmd)
                self.shutdownhook()
                

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass