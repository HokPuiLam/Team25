#not finished, just copied move_circle.py
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Circle:

    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_circle', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the 'move_circle' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            # specify the radius of the circle:
            path_rad = 0.5 # m
            # linear velocity must be below 0.26m/s:
            lin_vel = 0.1 # m/s

            self.vel_cmd.linear.x = lin_vel
            self.vel_cmd.angular.z = lin_vel / path_rad # rad/s

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass