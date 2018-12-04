#!/usr/bin/env python
# Software License Agreement (BSD License)
import sys
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from predict import *
from util import *


def process_depth(msg):
    bridge = CvBridge()


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
    toleration = 0.03
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


def closest(vec, mat,thresh):
    res = np.zeros(mat.shape[0])
    ans = []
    for i in range(mat.shape[0]):
        vec2 = mat[i,:]
        from numpy.linalg import norm
        dist1 = norm(vec[:3]-vec2[:3])
        dist2 = norm(vec[-3:]-vec2[-3:])
        #print(dist)
        if dist1<thresh and dist2<thresh:
            ans = ans + [i]

    return ans

def nearest_predict(vec,mat,pos):
    vec = vec.astype(float)
    mat = mat.astype(float)    
    min_val = sum(vec)
    ans = 0
    
    for i in range(mat.shape[0]):
        vec2 = mat[i,:]
        diff = abs(vec-vec2)
        # compute only the downward part
        diff = diff[int(len(diff)*0.5):]
        dt = np.sum(diff)
        #dt = np.sum(abs(vec-vec2))
        if dt<min_val:
            min_val=dt
            ans = i
    # vis_depth(mat[ans])
    # find top answer's indices in mat
    return pos[ans], mat[ans]

def nearest_predict_prev(vec,mat,pos):
    vec = vec.astype('float')
    mat = mat.astype('float')
    res = np.zeros(mat.shape[0])
    for i in range(mat.shape[0]):
        vec2 = mat[i,:]
        diff = abs(vec-vec2)
        diff = diff[int(len(diff)*0.5):]
        dt = np.sum(diff)
        #dt = np.sum(abs(vec-vec2))
        res[i]=dt
    # find top answer's indices in mat
    ans = np.argsort(res)[:20]
    return np.mean(pos[ans], axis=0), mat[ans[0]]

def find_target(depth, hint, task_id):
    hint = hint[-3:]+hint[:3]
    cands = closest(vec=hint, mat=hint_sim, thresh=0.1)
    print("cands",len(cands))
    if (len(cands)>0):
        pred, vec = nearest_predict(vec=depth, mat=depth_sim[cands], pos=pos_sim[task_id][cands])
        pred = pred.tolist()
        pred = pred[-3:]+pred[:3]
        return pred
    print("no solution!")
    hint = hint[-3:]+hint[:3]
    return hint


if __name__ == '__main__':

    # data_name = sys.argv[1]
    sim_dir = './config'
    depth_sim = np.load(os.path.join(sim_dir,'depth_sim_bg.npy'))
    hint_sim = np.load(os.path.join(sim_dir,'handles.npy'))[:,-6:]

    pos_sim = list()
    pos_sim.append(np.load(os.path.join(sim_dir,'handles-arc.npy')))
    pos_sim.append(np.load(os.path.join(sim_dir,'handles-flat.npy')))
    pos_sim.append(np.load(os.path.join(sim_dir, 'handles-twist.npy')))

    print("loaded", depth_sim.shape, len(pos_sim), hint_sim.shape)
    for i in range(len(pos_sim)):
        print(len(pos_sim[i]))

    task_id = 0

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
        try:
            if have_im:
                have_im = False
                # have target_feat
                vec = depth_feature(im)
                print"current_pos", current_pos
                target_pos = find_target(depth=vec,hint=current_pos.tolist(), task_id=task_id)
                print "target"
                print target_pos
                motion = np.array(target_pos) - np.array(current_pos)
                motion = validate_motion(motion)
                vel.data = motion
                print "motion", motion
                pub.publish(vel)

            else:
                print 'idle state'
                vel.data = np.zeros(6)
                pub.publish(vel)
            rate.sleep()

        except KeyboardInterrupt:
        	print "except catched, task changed"
        	task_id = (task_id+1)%3
            
