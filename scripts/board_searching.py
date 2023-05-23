#!/usr/bin/env python

import sys
import rospy

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult


def move(T):
    

def handle_move(req):
    f = move(req)

def joint_small_traj_server():
    rospy.init_node("board_search")
    s = rospy.Service("slight_planar_move", slight_planar_move, handle_move)
    rospy.spin()

