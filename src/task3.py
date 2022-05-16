#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class task3():

    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task3', anonymous=True)
        self.rate = rospy.Rate(10) # hz


        self.vel_cmd = Twist()
        self.right_arc = np.zeros(360)
        self.front_arc = np.zeros(360)
        self.ctrl_c = False
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the 'move_circle' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True

    def callback_lidar(self, lidar_data):
        """Obtain a subset of the LaserScan.ranges array corresponding to a +/-10 degree arc in front of it.
        Convert this subset to a numpy array to allow for more advanced processing."""
        
        #taken from a lab
        left_arc = lidar_data.ranges[0:91]
        right_arc = lidar_data.ranges[-90:]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])


    def main_loop(self):
        while not self.ctrl_c:
            min_right = np.amin(self.front_arc[120:140])
            min_front = np.amin(self.front_arc[80:100])

            if (min_front < 0.1):
                self.vel_cmd.angular.z = 1.3
                self.vel_cmd.linear.x = -0.1
                print("reversing")
            elif (min_front < 0.4):
                self.vel_cmd.angular.z = 1.3
                self.vel_cmd.linear.x = 0.00
                print("reorienting")
                       
            elif (min_right > 0.33 and min_right < 0.43 and min_front >= 0.4):
                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0.26
                print("forward")

            elif(min_right >= 0.43 and min_front >= 0.4):
                self.vel_cmd.angular.z = -0.90
                self.vel_cmd.linear.x = 0.26
                print("right")
            
            elif(min_right <= 0.33 and min_front >= 0.4):
                self.vel_cmd.angular.z = 0.90
                self.vel_cmd.linear.x = 0.26
                print("left")

            
            self.pub.publish(self.vel_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = task3()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass 