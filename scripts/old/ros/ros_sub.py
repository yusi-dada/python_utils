#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import pickle
import rospy
import roslib.packages
from moveit_msgs.msg import DisplayTrajectory

def callback(data):
    packagePath = roslib.packages.get_pkg_dir("python_utils")
    filePath = os.path.join(packagePath,'src','singer.pickle')

    with open(filePath, 'wb') as f:
        pickle.dump(data, f)    
    rospy.loginfo("save message")


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()