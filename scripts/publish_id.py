#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import rospy
from std_msgs.msg import String, Float64MultiArray, Int16
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from predict import *
from util import *



if __name__ == '__main__':


    pub = rospy.Publisher('/task_id', Int16, queue_size=10)


    rospy.init_node('pub', anonymous=True)


    task_id = Int16()
    task_id.data = 1
    rospy.sleep(12)
    pub.publish(task_id)

    task_id.data = 0
    rospy.sleep(12)
    pub.publish(task_id)


            
