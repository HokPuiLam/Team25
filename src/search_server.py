#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import time
# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal
from sensor_msgs.msg import LaserScan
# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.lidar_subscriber = self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        #zeroing robot pose
        self.x0 = 0.0
        self.y0 = 0.0

        #zeroing lidar
        self.front_min = 0
        self.right_min = 0
        self.left_min = 0
        
    
    def scan_callback(self, scan_data):
        f_left_arc = scan_data.ranges[0:27]
        f_right_arc = scan_data.ranges[-27:]
        right_arc = np.array(scan_data.ranges[320:350])
        left_arc = np.array(scan_data.ranges[20:50])
        front_arc = np.array(f_left_arc[::-1] + f_right_arc[::-1])
        self.front_min = np.amin(front_arc)
        self.right_min = np.amin(right_arc)
        self.left_min = np.amin(left_arc)

    
    def action_server_launcher(self, goal: SearchGoal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 10:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s "
                f"and stop {goal.approach_distance:.2f}m "
                f"infront of any obstacles")

        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        while True:
            #front is clear
            print(goal.approach_distance)
            if self.front_min > goal.approach_distance:
                
                # #both sides are clear, wiggle for more responsiveness
                if self.right_min > goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.set_move_cmd(0.26, 0.70)
                    self.vel_controller.publish()
                    self.vel_controller.stop()
                    self.vel_controller.set_move_cmd(0.26, -0.70)
                    self.vel_controller.publish()
                    print("all clear")

                #but right isn't clear
                if self.right_min < goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.stop()
                    self.vel_controller.set_move_cmd(0.26, 1)
                    self.vel_controller.publish()
                    print("right not clear")

                #but left isn't clear
                elif self.right_min > goal.approach_distance and self.left_min < goal.approach_distance:
                    self.vel_controller.stop()
                    self.vel_controller.set_move_cmd(0.26, 1)
                    self.vel_controller.publish()
                    print("left not clear")

                #both sides are not clear
                else:
                    self.vel_controller.set_move_cmd(0.26, 0)
                    self.vel_controller.publish()
                    print("only front clear")

            #front is not clear
            else:
                #but both sides are clear
                if self.right_min > goal.approach_distance and self.left_min > goal.approach_distance:
                    #and left side is closer to an obstacle, reverse and spin right
                    if self.right_min > self.left_min:
                        self.vel_controller.stop()
                        self.vel_controller.set_move_cmd(-0.1, -1)
                        self.vel_controller.publish()
                        print("front not clear")
                    #else, reverse and spin left
                    else:
                        self.vel_controller.stop()
                        self.vel_controller.set_move_cmd(-0.1, 1)
                        self.vel_controller.publish()
                        print("front not clear")

                #but the left isn't clear, so spin right
                elif self.right_min > goal.approach_distance and self.left_min < goal.approach_distance:
                    self.vel_controller.stop()
                    self.vel_controller.set_move_cmd(0, -1)
                    self.vel_controller.publish()
                    print("front left not clear")
                
                #but the right isn't clear, so spin left
                elif self.right_min < goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.stop()
                    self.vel_controller.set_move_cmd(0, 1)
                    self.vel_controller.publish()
                    print("front right not clear")

                #but no sides are clear
                else:
                    #and left side is closer to an obstacle, spin right
                    if self.right_min > self.left_min:
                        self.vel_controller.stop()
                        self.vel_controller.set_move_cmd(0, -1.82)
                        self.vel_controller.publish()
                        print("not clear")
                        
                    #and right side is closer to an obstacle, spin left
                    else:
                        self.vel_controller.stop()
                        self.vel_controller.set_move_cmd(0, 1.82)
                        self.vel_controller.publish()
                        print("not clear")
            self.feedback.current_distance_travelled = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            self.actionserver.publish_feedback(self.feedback)


"""         if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop() """
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()