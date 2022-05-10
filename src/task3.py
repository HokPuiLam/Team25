#!/usr/bin/env python3
#not finished, just copied move_circle.py


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
        self.front_arc_for_reverse = np.zeros(360)
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

        self.right_arc = np.array(lidar_data.ranges[270:300])
        self.front_arc = np.array(lidar_data.ranges[-15:] + lidar_data.ranges[15:0])



    def main_loop(self):
        while not self.ctrl_c:
            min_right = np.amin(self.right_arc)
            min_front = np.amin(self.front_arc)
            if (min_front > 0.45):
                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0.13
                self.pub.publish(self.vel_cmd)
                print("front")

            elif (min_front < 0.2):
                self.vel_cmd.angular.z = 0.2
                self.vel_cmd.linear.x = -0.13
                self.pub.publish(self.vel_cmd)
            else:

                if(min_right < 0.3 and min_right < 0.4):
                    self.vel_cmd.linear.x = 0.13
                    self.vel_cmd.angular.z = 0.00
                    self.pub.publish(self.vel_cmd)
                    print("actual front")
                elif (min_right < 0.3):
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 1.82
                    self.pub.publish(self.vel_cmd)
                    print("spin right")
                elif(min_right > 0.4):
                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = -1.82
                    self.pub.publish(self.vel_cmd) 
                    print("spin left")
            
            # if (min_front < 0.45):
            #     self.vel_cmd.angular.z = 0
            #     self.vel_cmd.linear.x = 0
            #     self.pub.publish(self.vel_cmd)
            #     self.vel_cmd.angular.z = 1.50
            #     self.pub.publish(self.vel_cmd)
            # else:
            #     print("min_right")
            #     print(min_right)
            #     if (min_right > 0.4 and min_right < 0.5):
            #         self.vel_cmd.angular.z = 0
            #         self.vel_cmd.linear.x = 0.15
            #         self.pub.publish(self.vel_cmd)
            #     elif(min_right < 0.4):
            #         self.vel_cmd.angular.z = 0.9
            #         self.vel_cmd.linear.x = 0.15
            #         self.pub.publish(self.vel_cmd)
                    
            #         print("left")
            #     elif(min_right > 0.5):
            #         self.vel_cmd.linear.x = 0.15
            #         self.vel_cmd.angular.z = -0.9
            #         self.pub.publish(self.vel_cmd)
            #         print(min_right)
            #         print("right")
            
        self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = task3()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass