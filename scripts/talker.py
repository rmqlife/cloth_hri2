#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import wrinkle2
import regression


def process_depth(msg):
	bridge = CvBridge()
	global have_im; have_im = True
	global im; im = bridge.imgmsg_to_cv2(msg)

def nearest_predict(vec,mat,pos):
    vec = vec.astype(int)
    mat = mat.astype(int)
    res = np.zeros(mat.shape[0])
    for i in range(mat.shape[0]):
        vec2= mat[i,:]
        dt = np.sum(abs(vec-vec2))
        res[i]=dt
    # find top answer's indices in mat
    ans = np.argsort(res)[:5]
    print ans
    print pos.shape
    return np.mean(pos[ans,:], axis=0)

def depth_feature(im, target_size=(64,64)):
    d = cv2.resize(im, target_size)
    return d.reshape(-1,)
    
def process_pos(msg):
	global have_current_pos; have_current_pos = True
	global current_pos; current_pos = np.array(msg.data)


def validate_motion(motion):
	print 'validate', max(abs(motion))
	toleration = 0.04
	if max(abs(motion))>toleration:
		print "bad"
		motion = motion*toleration/max(abs(motion))
	return motion


def validate_pos(current_pos,pos):
    upper = np.zeros(6)
    lower = np.zeros(6)
    for i in range(pos.shape[1]):
        upper[i] = max(pos[:,i])+0.05
        lower[i] = min(pos[:,i])-0.05
    for i in range(len(current_pos)):
        if current_pos[i]<lower[i]:
            print 'lower',lower
            return False
        if current_pos[i]>upper[i]:
            print 'upper', upper
            return False
    return True
	

if __name__ == '__main__':

	# data_name = sys.argv[1]
	data = np.load('data.npz')
	tt_pos = data['pos']
	feat = np.load('depth1.npy')
	print tt_pos.shape
	print feat.shape
	
	rr = [0.48943184,0.1678617,0.47914139]
    rl = [0.4918203,-0.11984081,0.47457296]

	target_pos = rr + rl
	global have_current_pos; have_current_pos = False
	global have_im; have_im = False
	global current_pos,im

	pub = rospy.Publisher('/yumi/ikSloverVel_controller/command', Float64MultiArray, queue_size=10)

	rospy.Subscriber('/yumi/ikSloverVel_controller/ee_cart_position', Float64MultiArray , process_pos, queue_size = 2)
	rospy.Subscriber('/camera/image/registered_depth__611205001943',Image, process_depth, queue_size=2)
	rospy.sleep(1)	

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	vel = Float64MultiArray()
	vel.data = np.zeros(6)
	while not rospy.is_shutdown():
		if have_im: 
			have_im = False
			# have target_feat
			vec = depth_feature(im)
			pos = nearest_predict(vec=vec, mat=feat, pos=tt_pos)
			print "pos"
			print pos
			print current_pos
			#hist = wrinkle2.xhist(im)
			#goal = goals[0]
			#target_feat = feat[goal,:]
			#hist = np.array(target_feat) - np.array(hist)
			#motion = model.predict(hist.reshape((1,-1))).ravel()
			motion = 0.3*[target - pos]
			motion = 0.3*motion
			motion = validate_motion(motion)
			vel.data = motion
			print "motion", motion
			pub.publish(vel)

		else: 
			print 'idle state'
			vel.data = np.zeros(6)
			pub.publish(vel)
		rate.sleep()
