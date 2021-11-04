#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import pickle
import rospy
import roslib.packages
from moveit_msgs.msg import DisplayTrajectory

if __name__ == '__main__':

    rospy.init_node('chatter', anonymous=True)
    pub = rospy.Publisher("move_group/display_planned_path", DisplayTrajectory, queue_size=2)

    packagePath = roslib.packages.get_pkg_dir("python_utils")
    filePath = os.path.join(packagePath,'src','singer.pickle')

    if os.path.exists(filePath):
        with open(filePath, 'rb') as f:
            data = pickle.load(f)
    

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(data)
        r.sleep()
