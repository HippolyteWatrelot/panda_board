#!/usr/bin/env python

import os
import time
import numpy as np
import rospy
import actionlib
import sys
from trajectory_types import *
from franka import Panda
from self_panda import Self_Panda

from panda_board.srv import (
    ExecuteJointTrajectory,
    BoardLocalization,
    FeaturesLocalization,
    PressRedButton,
    PressBlueButton,
    AimRedPlug,
    AimBlackPlug,
    Sliding,
    Trap
from actionlib import SimpleActionClient
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult
)
from sensor_msgs import JointState
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from franka_gripper.msg import HomingActionGoal, GraspActionGoal, MoveActionGoal, StopActionGoal


current_end_effector_position, current_end_effector_orientation = np.zeros(3), np.array([0, 1, 0, 0])
current_end_effector_transmatrix = np.diag(np.ones(4))

current_board_position, current_board_orientation = np.zeros(3), np.zeros(4)
current_slider_position, current_slider_orientation = np.zeros(3), np.zeros(4)
current_trap_position, current_trap_orientation = np.zeros(3), np.zeros(4)
current_pod_position, current_pod_orientation = np.zeros(3), np.zeros(4)
current_joints_pose = np.zeros(7)
current_red_button_position, current_blue_button_position = np.zeros(2)
current_red_plug_position, current_black_plug_position = np.zeros(2)
current_slider_state, current_trap_state = -1, -1
command, status = 0, 0





def pose_to_transmatrix(p, q):
    return np.array([[2*(q[0]**2 + q[1]**2) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2]), p[0]],
                     [2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]**2 + q[2]**2) - 1, 2*(q[2]*q[3] - q[0]*q[1]), p[1]],
                     [2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]**2 + q[3]**2) - 1, p[2]],
                     [0, 0, 0, 1]])




def handle_current_pose(cart_pose):
    global current_end_effector_position, current_end_effector_orientation
    current_end_effector_position[0] = cart_pose.pose.position.x
    current_end_effector_position[1] = cart_pose.pose.position.y
    current_end_effector_position[2] = cart_pose.pose.position.z
    current_end_effector_orientation[0] = cart_pose.pose.orientation.w
    current_end_effector_orientation[1] = cart_pose.pose.orientation.x
    current_end_effector_orientation[2] = cart_pose.pose.orientation.y
    current_end_effector_orientation[3] = cart_pose.pose.orientation.z

def handle_current_joints_states(joints_pose):
    global current_joints_pose = joints_pose.position

def handle_board_pose(board_pose):
    global current_board_position, current_board_orientation
    current_board_position[0] = board_pose.pose.position.x
    current_board_position[1] = board_pose.pose.position.y
    current_board_position[2] = board_pose.pose.position.z
    current_board_orientation[0] = board_pose.pose.orientation.w
    current_board_orientation[1] = board_pose.pose.orientation.x
    current_board_orientation[2] = board_pose.pose.orientation.y
    current_board_orientation[3] = board_pose.pose.orientation.z

def handle_red_button_position(position):
    global current_red_button_position
    current_red_button_position[0] = position.point.x
    current_red_button_position[1] = position.point.y

def handle_blue_button_position(position):
    global current_blue_button_position
    current_blue_button_position[0] = position.point.x
    current_blue_button_position[1] = position.point.y

def handle_red_plug_position(position):
    global current_red_plug_position
    current_red_plug_position[0] = position.point.x
    current_red_plug_position[1] = position.point.y

def handle_black_plug_position(position):
    global current_black_plug_position
    current_black_plug_position[0] = position.point.x
    current_black_plug_position[1] = position.point.y

def handle_slider_pose(slider_pose):
    global current_slider_position, current_slider_orientation
    current_slider_position[0] = slider_pose.pose.position.x
    current_slider_position[1] = slider_pose.pose.position.y
    current_slider_position[2] = slider_pose.pose.position.z
    current_slider_orientation[0] = slider_pose.pose.orientation.w
    current_slider_orientation[1] = slider_pose.pose.orientation.x
    current_slider_orientation[2] = slider_pose.pose.orientation.y
    current_slider_orientation[3] = slider_pose.pose.orientation.z

def handle_slider_state(msg):
    global current_slider_state = msg.data

def handle_trap_pose(trap_pose):
    global current_trap_position, current_trap_orientation
    current_trap_position[0] = trap_pose.pose.position.x
    current_trap_position[1] = trap_pose.pose.position.y
    current_trap_position[2] = trap_pose.pose.position.z
    current_trap_orientation[0] = trap_pose.pose.orientation.w
    current_trap_orientation[1] = trap_pose.pose.orientation.x
    current_trap_orientation[2] = trap_pose.pose.orientation.y
    current_trap_orientation[3] = trap_pose.pose.orientation.z

def handle_trap_state(msg):
    global current_trap_state = msg.data

def handle_pod_pose(pod_pose):
    global current_pod_position, current_pod_orientation
    current_pod_position[0] = pod_pose.pose.position.x
    current_pod_position[1] = pod_pose.pose.position.y
    current_pod_position[2] = pod_pose.pose.position.z
    current_pod_orientation[0] = pod_pose.pose.orientation.w
    current_pod_orientation[1] = pod_pose.pose.orientation.x
    current_pod_orientation[2] = pod_pose.pose.orientation.y
    current_pod_orientation[3] = pod_pose.pose.orientation.z

def handle_command(msg):
    command = msg.data 

def handle_status(msg):
    status = msg.data







def manager(message):
    current_end_effector_transmatrix = pose_to_transmatrix(current_end_effector_position, current_end_effector_orientation)






def handle_board_action(req):

def handle_features_action(req):

def handle_red_button_action(req):

def handle_blue_button_action(req):

def handle_red_plug_action(req):

def handle_black_plug_action(req):

def handle_sliding_action(req):

def handle_trap_action(req):




def board_data_manager():

    rospy.init_node("board_data_manager")

    sub_current_pose = rospy.Subscriber("/cartesian_pose", PoseStamped, handle_current_pose)
    sub_current_joints_poses = rospy.Subscriber("/joint_states", JointState, handle_current_joints_states)
    sub_board_pose = rospy.Subscriber("/panda_board/board_estimated_pose", PoseStamped, handle_board_pose)
    sub_red_button_position = rospy.Subscriber("/panda_board/red_button_position", PointStamped, handle_red_button_position)
    sub_blue_button_position = rospy.Subscriber("/panda_board/red_button_position", PointStamped, handle_blue_button_position)
    sub_red_plug_position = rospy.Subscriber("/panda_board/red_plug_position", PointStamped, handle_red_plug_position)
    sub_black_plug_position = rospy.Subscriber("/panda_board/black_plug_position", PointStamped, handle_black_plug_position)
    sub_slider_pose = rospy.Subscriber("/panda_board/slider_pose", PoseStamped, handle_slider_pose)
    sub_slider_state = rospy.Subscriber("/panda_board/slider_state", Int16, handle_slider_state)
    sub_trap_pose = rospy.Subscriber("/panda_board/trap_pose", PoseStamped, handle_trap_pose)
    sub_trap_state = rospy.Subscriber("/panda_board/slider_state", Int16, handle_trap_state)
    sub_pod_pose = rospy.Subscriber("/panda_board/pod_pose", PoseStamped, handle_pod_pose)
    command_sub = rospy.Subscriber("/panda_board/command_ind", Int16, handle_command)
    status_sub = rospy.Subscriber("/panda_board/status_ind", Int16, handle_status)

    grasp_pub = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=0)
    stop_pub = rospy.Publisher("/franka_gripper/stop/goal", StopActionGoal, queue_size=0)
    move_pub = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal, queue_size=0)
    homing_pub = rospy.Publisher("/franka_gripper/homing/goal", HomingActionGoal, queue_size=0)

    s1 = rospy.Service("board_localization", BoardLocalization, handle_board_action)
    s2 = rospy.Service("features_localization", FeaturesLocalization, handle_features_action)
    s3 = rospy.Service("press_red_button", PressRedButton, handle_red_button_action)
    s4 = rospy.Service("press_blue_button", PressBlueButton, handle_blue_button_action)
    s5 = rospy.Service("aim_red_plug", AimRedPlug, handle_red_plug_action)
    s6 = rospy.Service("aim_black_plug", AimBlackPlug, handle_black_plug_action)
    s7 = rospy.Service("sliding", Sliding, handle_sliding_action)
    s8 = rospy.Service("trap", Trap, handle_trap_action)

    rospy.sleep(1)
    rospy.spin()


if __name__ == "__main__":
    board_data_manager()

