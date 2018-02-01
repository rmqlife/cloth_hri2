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
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from predict import *
from util import *
class Finder:
    def __init__(self, depth_sim, pos_sim):
        self.use_simple=True
        self.debug=False
        self.pos_sim = pos_sim
        self.depth_sim = depth_sim
        if self.use_simple:
            self.nn = Nearest(depth_sim)
        
        pass
    
    def get_target(self, vec):
        if self.use_simple:
            pred, ans = nearest_predict(vec=vec
                                      ,mat=self.depth_sim
                                      ,pos=self.pos_sim)
        else:
            dist, ans = self.nn.findNeigh(depth_orig[i,:])
            pred = pos_sim[ans]
        if self.debug:
            vis_depth(depth_orig[i,:])
            vis_depth(depth_sim[ans,:])
        
        pred = pred.tolist()
        pred = pred[-3:]+pred[:3]
        return pred, self.depth_sim[ans]


def process_depth(msg):
    bridge = CvBridge()
    global have_im; have_im = True
    global im; im = bridge.imgmsg_to_cv2(msg)


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
    depth_sim = np.load('depth_sim_bg.npy')
    pos_sim = np.load('expert.npy')
    finder = Finder(depth_sim, pos_sim)
    

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
            target_pos = finder.get_target(vec)
            print "pos"
            print target_pos
            print current_pos
            #hist = wrinkle2.xhist(im)
            #goal = goals[0]
            #target_feat = feat[goal,:]
            #hist = np.array(target_feat) - np.array(hist)
            #motion = model.predict(hist.reshape((1,-1))).ravel()
            motion = target_pos - current_pos
            motion = validate_motion(motion)
            vel.data = motion
            print "motion", motion
            pub.publish(vel)

        else: 
            print 'idle state'
            vel.data = np.zeros(6)
            pub.publish(vel)
        rate.sleep()
