#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import numpy as np
import math
from math import sqrt, pow, pi
from sensor_msgs.msg import LaserScan

class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
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
        self.search = False
        self.detected = False
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
        self.cz = m['m01'] / (m['m00'] + 1e-5)

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
    def scan_callback(self, scan_data):
        f_left_arc = scan_data.ranges[0:27]
        f_right_arc = scan_data.ranges[-27:]
        right_arc = np.array(scan_data.ranges[320:350])
        left_arc = np.array(scan_data.ranges[20:50])
        front_arc = np.array(f_left_arc[::-1] + f_right_arc[::-1])
        self.front_min = np.amin(front_arc)
        self.right_min = np.amin(right_arc)
        self.left_min = np.amin(left_arc)

    def rotate_right(self):
        print('rotate right')
        self.robot_controller.set_move_cmd(0.0, -0.5)   
        self.robot_controller.publish()
        rospy.sleep(3.2)
        self.robot_controller.stop()

    def rotate_left(self):
        print('rotate left')
        self.robot_controller.set_move_cmd(0.0, 0.5)   
        self.robot_controller.publish()
        rospy.sleep(3.2)
        self.robot_controller.stop()

    def forward(self):
        print("forward")
        self.robot_controller.set_move_cmd(0.2, 0.0)
        self.robot_controller.publish()
        rospy.sleep(0.5)
        self.robot_controller.stop()




    def move_around(self):
                # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        #while True:
            #front is clear
            #print(0.6000000238418579)
            #print(self.color)
            #self.init_color()
            # if self.detected_beacon() == True:
            #     break
        if self.front_min > 0.6000000238418579:
            
            # #both sides are clear, wiggle for more responsiveness
            if self.right_min > 0.6000000238418579 and self.left_min > 0.6000000238418579:
                self.robot_controller.set_move_cmd(0.26, 0.70)
                self.robot_controller.publish()
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.26, -0.70)
                self.robot_controller.publish()
                #print("all clear")

            #but right isn't clear
            if self.right_min < 0.6000000238418579 and self.left_min > 0.6000000238418579:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.26, 1)
                self.robot_controller.publish()
                #print("right not clear")

            #but left isn't clear
            elif self.right_min > 0.6000000238418579 and self.left_min < 0.6000000238418579:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0.26, 1)
                self.robot_controller.publish()
                #print("left not clear")

            #both sides are not clear
            else:
                self.robot_controller.set_move_cmd(0.26, 0)
                self.robot_controller.publish()
                #print("only front clear")

        #front is not clear
        else:
            #but both sides are clear
            if self.right_min > 0.6000000238418579 and self.left_min > 0.6000000238418579:
                #and left side is closer to an obstacle, reverse and spin right
                if self.right_min > self.left_min:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(-0.1, -1)
                    self.robot_controller.publish()
                    #print("front not clear")
                #else, reverse and spin left
                else:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(-0.1, 1)
                    self.robot_controller.publish()
                    #print("front not clear")

            #but the left isn't clear, so spin right
            elif self.right_min > 0.6000000238418579 and self.left_min < 0.6000000238418579:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, -1)
                self.robot_controller.publish()
                #print("front left not clear")
            
            #but the right isn't clear, so spin left
            elif self.right_min < 0.6000000238418579 and self.left_min > 0.6000000238418579:
                self.robot_controller.stop()
                self.robot_controller.set_move_cmd(0, 1)
                self.robot_controller.publish()
                #print("front right not clear")

            #but no sides are clear
            else:
                #and left side is closer to an obstacle, spin right
                if self.right_min > self.left_min:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, -1.82)
                    self.robot_controller.publish()
                    #print("not clear")
                    
                #and right side is closer to an obstacle, spin left
                else:
                    self.robot_controller.stop()
                    self.robot_controller.set_move_cmd(0, 1.82)
                    self.robot_controller.publish()
                    #print("not clear")


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
                #print("self search true")
                self.search = True
                #print(self.search)
                break

    def detected_beacon(self):
        color_threshold = {
            "Blue": [(115, 224, 100), (130, 255, 255)],
            "Red": [(0, 185, 100), (10, 255, 255)],
            "Green": [(25, 150, 100), (70, 255, 255)],
            "Turquoise": [(75, 150, 100), (100, 255, 255)]
        }

        for colorname, (lower, upper) in color_threshold.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower, upper)
            if mask.any():
                print("TARGET DETECTED: Beaconing initiated.{}.".format (colorname))
                self.detected == True
                self.robot_controller.stop()
                #print(self.detected)
                return True
                break


    def main(self):
        while not self.ctrl_c:
            # if self.detected == True:
            #         #print("stop")
            #     print("detected")
            #     self.robot_controller.stop()
            #     self.shutdown_ops()
            #     self.ctrl_c == True
            # if self.search == False:
            #     rospy.sleep(1)
            #     self.rotate_right()
            #     self.init_color()
            #     self.rotate_left()
            #     self.robot_controller.stop()
            # if self.search == True:
            #     #print("self turn true")
            #     #print('fast')
            #     self.move_around()
            #     #self.ctrl_c = True









            # if self.move_rate == 'fast':
            #     self.move_around()
            # elif self.move_rate == 'stop':
            #     self.robot_controller.stop()
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






            # if self.stop_counter > 0:
            #     self.stop_counter -= 1
            if self.search == False:
                rospy.sleep(1)
                self.rotate_right()
                self.init_color()
                self.rotate_left()
                self.forward()
                self.robot_controller.stop()
            else:
                #print("testing")
                if self.m00 > self.m00_min and self.detected == False:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            if int(self.cz) > 180 and int(self.cz) < 220:
                                self.move_rate = 'stop'  
                                self.find_target = True
                                print("BEACON DETECTED: Beaconing initiated.")
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'

                if self.move_rate == 'fast':
                    # print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                    # self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    self.move_around()
                elif self.move_rate == 'slow':
                    if self.cy <= 560-100:
                        self.robot_controller.set_move_cmd(0.1, 0.2)
                        self.robot_controller.publish()
                        print("slow left")
                    elif self.cy > 560+100:
                        self.robot_controller.set_move_cmd(0.1, -0.2)
                        self.robot_controller.publish()
                        print("slow right")
                    #print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    #self.robot_controller.set_move_cmd(0.0, 0.1)
                elif self.move_rate == 'stop' and self.stop_counter > 0:
                    print("stop")
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                #else:
                    #print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    #self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                
                self.robot_controller.publish()
                self.rate.sleep()





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