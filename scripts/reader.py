#!/usr/bin/env python
# This script is to record the training data into a file
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import wrinkle2
import regression
import time

def process_rgb(msg):
	bridge = CvBridge()
	global have_im; have_im = True
	global im; im = bridge.imgmsg_to_cv2(msg)

def process_pos(msg):
	global have_current_pos; have_current_pos = True
	global current_pos; current_pos = np.array(msg.data)


if __name__ == '__main__':
	global have_current_pos; have_current_pos = False
	global have_im; have_im = False
	global current_pos,im

	rospy.Subscriber('/yumi/ikSloverVel_controller/ee_cart_position', Float64MultiArray , process_pos, queue_size = 1)
	rospy.Subscriber('/camera/image/rgb_611205001943',Image, process_rgb, queue_size = 1)
	rospy.sleep(1)	

	rospy.init_node('reader', anonymous=True,  disable_signals=True)
	rate = rospy.Rate(10) # 10hz
	
	start_time = time.time()
	pos = np.array([])
	feat = np.array([])
	while not rospy.is_shutdown():
		try:
			if have_current_pos and have_im:
				print 'time', time.time()-start_time
				print current_pos
				hist = wrinkle2.xhist(im)
				hist = np.array(hist)
				pos = np.vstack((pos,current_pos)) if pos.size else current_pos
				feat = np.vstack((feat,hist)) if feat.size else hist
				have_current_pos = False
				have_im = False
			rate.sleep()
		except KeyboardInterrupt:
			print "except catched", len(pos)
			time_str = time.strftime("%m%d-%H%M")
			np.savez('data_'+time_str+'.npz',pos=pos, feat=feat)
			break
