#! /usr/bin/env python3

import rospy
import numpy as np
import os

from panda_board.srv import ExecuteJointTrajectory

from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectoryPoint


nbr = 4
traj_left = []
traj_right = []
path = os.path.abspath(os.getcwd()) + '/src/franka_ros/panda_board/data/'
for i in range(nbr):
    traj_left.append(np.load(path + "data_left_%s.npy"%str(i+1)))
    traj_right.append(np.load(path + "data_left_%s.npy"%str(i+1)))



def execute_joint_trajectory_client(gripper_time):
    message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
    rospy.wait_for_service("execute_joint_traj_service")
    try:
        traj = rospy.ServiceProxy("execute_joint_traj_service", ExecuteJointTrajectory)
        traj(gripper_time)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


if __name__ == "__main__":
    joint_pose_pub = rospy.Publisher("/panda_board/candidate_joint_trajectory_point", JointTrajectoryPoint, queue_size=1)
    action = 0
    k = 0
    if len(sys.argv) != 2:
        pass
    else:
        action = int(sys.argv[1])
    while action not in ['0', '1']:
        action = input("What action do you want to do ? (0: Close left, 1: Close right)")
    action = int(action)
    gripper_time = None
    if action == 0:
        for i in range(nbr):
            jointpoint = JointTrajectoryPoint()
            for j in range(len_left[i]):
                jointpoint.positions = traj_left[i][j, :]
                jointpose_pub.publish(jointpoint)
            execute_joint_trajectory_client(gripper_time)
    else:
        for i in range(nbr):
            jointpoint = JointTrajectoryPoint()
            for j in range(len_right[i]):
                jointpoint.positions = traj_right[i][j, :]
                jointpose_pub.publish(jointpoint)
            execute_joint_trajectory_client(gripper_time)#! /usr/bin/env python3

import rospy
import numpy as np
import os

from panda_board.srv import ExecuteJointTrajectory

from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectoryPoint


nbr = 4
traj_left = []
traj_right = []
path = os.path.abspath(os.getcwd()) + '/src/franka_ros/panda_board/data/'
for i in range(nbr):
    traj_left.append(np.load(path + "data_left_%s.npy"%str(i+1)))
    traj_right.append(np.load(path + "data_left_%s.npy"%str(i+1)))



def execute_joint_trajectory_client(gripper_time):
    message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
    rospy.wait_for_service("execute_joint_traj_service")
    try:
        traj = rospy.ServiceProxy("execute_joint_traj_service", ExecuteJointTrajectory)
        traj(gripper_time)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


if __name__ == "__main__":
    joint_pose_pub = rospy.Publisher("/panda_board/candidate_joint_trajectory_point", JointTrajectoryPoint, queue_size=1)
    action = 0
    k = 0
    if len(sys.argv) != 2:
        pass
    else:
        action = int(sys.argv[1])
    while action not in ['0', '1']:
        action = input("What action do you want to do ? (0: Close left, 1: Close right)")
    action = int(action)
    gripper_time = None
    if action == 0:
        for i in range(nbr):
            jointpoint = JointTrajectoryPoint()
            for j in range(len_left[i]):
                jointpoint.positions = traj_left[i][j, :]
                jointpose_pub.publish(jointpoint)
            execute_joint_trajectory_client(gripper_time)
    else:
        for i in range(nbr):
            jointpoint = JointTrajectoryPoint()
            for j in range(len_right[i]):
                jointpoint.positions = traj_right[i][j, :]
                jointpose_pub.publish(jointpoint)
            execute_joint_trajectory_client(gripper_time)
