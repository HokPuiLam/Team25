#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import argparse
import roslaunch
# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from pathlib import Path

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.pic_path = Path.home().joinpath("catkin_ws/src/Team25/snaps/the_beacon.jpg")
        self.ctrl_c = False
        self.take_dummy_photo = True
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(10)

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.lidar_subscriber = self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)


        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        #zeroing robot pose
        self.x0 = 0.0
        self.y0 = 0.0

        #zeroing lidar
        self.front_min = 0
        self.right_min = 0
        self.left_min = 0

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.search = False
        self.detected = False
        self.lower = []
        self.upper = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.m00 = 0
        self.m00_min = 100000

        # Command-Line Interface:
        cli = argparse.ArgumentParser(description=f"Command-line interface for the 'argparse' node.")
        cli.add_argument("-target_colour", metavar="COL", type=String,
            default="blue", 
            help="The name of a colour (for example)")
       
        # obtain the arguments passed to this node from the command-line:
        self.args = cli.parse_args(rospy.myargv()[1:])
        
        self.target_colour = self.args.target_colour.data
        print ("colour targetted")
        print (self.target_colour)
        
    
    def scan_callback(self, scan_data):
        f_left_arc = scan_data.ranges[0:25]
        f_right_arc = scan_data.ranges[-25:]
        right_arc = np.array(scan_data.ranges[320:340])
        left_arc = np.array(scan_data.ranges[26:47])
        front_arc = np.array(f_left_arc[::-1] + f_right_arc[::-1])
        self.front_min = np.amin(front_arc)
        self.right_min = np.amin(right_arc)
        self.left_min = np.amin(left_arc)
    
    def init_color(self):
        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        # self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        # self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]
        color_threshold = {
            "blue": [(115, 224, 100), (130, 255, 255)],
            "red": [(0, 185, 100), (10, 255, 255)],
            "green": [(25, 150, 100), (70, 255, 255)],
            "turquoise": [(75, 150, 100), (100, 255, 255)],
            "purple":   ([145, 190, 100], [155, 255, 255]),
            "yellow": ([28, 180, 100], [32, 255, 255])
        }

        for color, (lower, upper) in color_threshold.items():
            lower = np.array(lower)
            upper = np.array(upper)
            if self.target_colour == color:
                self.color = color
                self.lower = lower
                self.upper = upper
                #print("self search true")
                self.search = True
                #print(self.search)

    def show_and_save_image(self, img):
        full_image_path = self.pic_path
        cv2.imwrite(str(full_image_path), img)

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        if (self.take_dummy_photo == True):
            self.show_and_save_image(self.cv_img)
            self.take_dummy_photo = False
        height, width, _ = self.cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2)) + 200

        crop_img = self.cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img

        # create a single mask to accommodate all four dectection colours:
        # for i in range(4):
        #     if i == 0:
        #         mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #         # print("mask123")
        #     else:
        #         mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #         # print("maskmasktest")

        if len(self.lower) > 0 and len(self.upper) > 0:
            self.mask = cv2.inRange(hsv_img, self.lower, self.upper)

        m = cv2.moments(self.mask)
            
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)
        self.cz = m['m01'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            #cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
            cv2.circle(crop_img, (int(self.cy), int(self.cz)), 10, (0, 0, 255), 2)
        
        cv2.waitKey(1)

    def shutdown_ops(self):
        self.vel_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def action_server_launcher(self, goal: SearchGoal):
        while not self.ctrl_c:
            self.init_color()
            


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


            # Get the current robot odometry:
            self.posx0 = self.tb3_odom.posx
            self.posy0 = self.tb3_odom.posy


            if self.m00 > self.m00_min and self.detected == False:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                        if int(self.cz) > 10 and int(self.cz) < 220: 
                            self.show_and_save_image(self.cv_img)

            #front is clear                                
            if self.front_min > goal.approach_distance:
                
                # #both sides are clear, wiggle for more responsiveness
                if self.right_min > goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.set_move_cmd(0.22, 0.3)
                    self.vel_controller.publish()
                    self.vel_controller.set_move_cmd(0.22, -0.30)
                    self.vel_controller.publish()
                    print("all clear")

                #but right isn't clear
                elif self.right_min < goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.set_move_cmd(0.15, 1)
                    self.vel_controller.publish()
                    print("right not clear")

                #but left isn't clear
                elif self.right_min > goal.approach_distance and self.left_min < goal.approach_distance:
                    self.vel_controller.set_move_cmd(0.15, -1)
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
                        self.vel_controller.set_move_cmd(0, -1.82)
                        self.vel_controller.publish()
                        print("front not clear")
                    #else, reverse and spin left
                    else:
                        self.vel_controller.set_move_cmd(0, 1.82)
                        self.vel_controller.publish()
                        print("front not clear")

                #but the left isn't clear, so spin right
                elif self.right_min > goal.approach_distance and self.left_min < goal.approach_distance:
                    self.vel_controller.set_move_cmd(-0.05, -1)
                    self.vel_controller.publish()
                    print("front left not clear")
                
                #but the right isn't clear, so spin left
                elif self.right_min < goal.approach_distance and self.left_min > goal.approach_distance:
                    self.vel_controller.set_move_cmd(-0.05, 1)
                    self.vel_controller.publish()
                    print("front right not clear")

                #but no sides are clear
                else:
                    #and left side is closer to an obstacle, spin right
                    if self.right_min > self.left_min:
                        self.vel_controller.set_move_cmd(-0.001, -1.82)
                        self.vel_controller.publish()

                        print("not clear, sr")
                        
                    #and right side is closer to an obstacle, spin left
                    else:
                        self.vel_controller.set_move_cmd(-0.001, 1.82)
                        self.vel_controller.publish()
                        print("not clear, sl")
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