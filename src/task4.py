#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move
import numpy as np
import math
from math import sqrt, pow, pi
class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(10)
        
        self.m00 = 0
        self.m00_min = 100000

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.turn = False
        self.color = ""
        self.lower = []
        self.upper = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        # self.mask = np.zeros((1080,1920,1), np.uint8)
        # self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        # self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        # self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
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

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow("cropped image", crop_img)
        cv2.waitKey(1)

    # def rotate(self, degree, speed):
    #     rospy.sleep(1)
    #     time_cal = math.radians(degree) / speed
    #     self.robot_controller.set_move_cmd(0.0, speed)    
    #     self.robot_controller.publish()
    #     time_cal = abs(time_cal)
    #     rospy.sleep(time_cal)
    #     self.robot_controller.stop()

    # def go_foward(self):
    #     self.robot_controller.set_move_cmd(0.2, 0)    
    #     self.robot_controller.publish()
    #     rospy.sleep(5)
    #     self.robot_controller.stop()

    def rotate(self):
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)   
        self.robot_controller.publish()



    def init_color(self):
        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        # self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        # self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]
        color_threshold = {
            "Blue": [(115, 224, 100), (130, 255, 255)],
            "Red": [(0, 185, 100), (10, 255, 255)],
            "Green": [(25, 150, 100), (70, 255, 255)],
            "Turquoise": [(75, 150, 100), (100, 255, 255)]
        }

        for color, (lower, upper) in color_threshold.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower, upper)
            if mask.any():
                self.color = color
                self.lower = lower
                self.upper = upper
                print("SEARCH INITIATED: The target beacon colour is {}.".format (self.color))
                self.turn = True
                break





    def main(self):
        while not self.ctrl_c:
            # if self.turn == False:
            #     self.rotate(100, 0.6)
            #     self.init_color()
            #     self.rotate(100, -0.6)
            #     self.go_foward()
            #     self.rotate(110, 0.6)
            #     self.turn = True
            # else:
            if self.turn == False:
                print("self turn false")
                self.rotate()
                self.init_color()
            else:
                print("self turn true")
                self.ctrl_c = True
            # if self.m00 > self.m00_min:
            #     # blob detected
            #     if self.cy >= 560-100 and self.cy <= 560+100:
            #         if self.move_rate == "slow":
            #             self.move_rate = "stop"
            #             self.stop_counter = 20
            #     else:
            #         self.move_rate = "slow"
            # else:
            #     self.move_rate = "fast"
                



            # if self.move_rate == "fast":
            #     print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            # elif self.move_rate == "slow":
            #     print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            # elif self.move_rate == "stop" and self.stop_counter > 0:
            #     print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
            #     self.robot_controller.set_move_cmd(0.0, 0.0)
            # else:
            #     print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
            #     self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            # self.robot_controller.publish()
            # self.rate.sleep()
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass