#! /usr/bin/env python3


import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from transform_utils.py import *


offset = 0.016
ee_cartesian_pose = PoseStamped()


Blist = np.array([[0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [np.sqrt(2)/2, -np.sqrt(2)/2, 0, 0.593*np.cos(np.pi/4), 0.593*np.cos(np.pi/4), 0.088],
                       [0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [-np.sqrt(2)/2, np.sqrt(2)/2, 0, -0.277*np.cos(np.pi/4), -0.277*np.cos(np.pi/4), 0],
                       [0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [-np.sqrt(2)/2, np.sqrt(2)/2, 0, 0.107*np.cos(np.pi/4), 0.107*np.cos(np.pi/4), -0.088],
                       [0, 0, 1, 0, 0, 0]]).T


Master_Tse = np.array([[np.sqrt(2) / 2,  np.sqrt(2) / 2,  0, 0.088], 
                       [np.sqrt(2) / 2, -np.sqrt(2) / 2,  0,     0], 
                       [             0,               0, -1, 0.926], 
                       [             0,               0,  0,     1]])


def transmatrix_to_posestamped():
    p = PoseStamped()
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        S = np.sqrt(1 + trace) * 2
        qw = S / 4
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = S / 4
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = S / 4
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = S / 4
    p.pose.position.x = R[0, 3]
    p.pose.position.y = R[1, 3]
    p.pose.position.z = R[2, 3]
    p.pose.orientation.w = qw
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    return pose



def handle_jointpose_to_posestamped(joints_pose):
    global pose
    cart_pose_pub = rospy.Publisher("/offset_cartesian_pose", PoseStamped, queue_size=500)
    thetalist = [joints_pose.position[i] for i in range(len(joints_pose.position))]
    transmatrix = FKinBody(Master_Tse, Blist, thetalist)
    ee_cartesian_pose = transmatrix_to_posestamped(transmatrix)
    cart_pose_pub.publish(ee_cartesian_pose)




def jointspose_to_cartpose_server():

    rospy.init_node("jointspose_to_cartpose_server")
    
    joints_pose_sub = rospy.Subscriber("/joint_states", JointState, handle_jointpose_to_posestamped)
 
    rospy.spin()


if __name__ == "__main__":
    jointspose_to_cartpose_server()
