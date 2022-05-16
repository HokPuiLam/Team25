#!/usr/bin/env python3

import roslaunch
import rospy
from pathlib import Path

map_path = Path.home().joinpath("catkin_ws/src/Team25/maps/task5_map")

rospy.init_node("map_getter", anonymous=True)
rate = rospy.Rate(1/5) # hz

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

while not rospy.is_shutdown():
    print(f"Saving map at time: {rospy.get_time()}...")

    node = roslaunch.core.Node(package="map_server",
                            node_type="map_saver",
                            args=f"-f {map_path}")
    process = launch.launch(node)
    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        pass