#! /usr/bin/env python3

import rospy
import numpy as np
import sys

from panda_board.srv import ExecuteJointTrajectory

from std_msgs.msg import Float32, Int16, String


def execute_joint_trajectory_client(gripper_time, indice):
    message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
    rospy.wait_for_service("execute_joint_traj_service")
    try:
        traj = rospy.ServiceProxy("execute_joint_traj_service", ExecuteJointTrajectory)
        traj(gripper_time, indice)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


if __name__ == "__main__":

    if len(sys.argv) == 1:
        gripper_time = -1
        while not isinstance(gripper_time, float) or gripper_time < 0:
            gripper_time = float(input("time for gripper to close : "))
    else:
        gripper_time = float(sys.argv[1])
    if len(sys.argv) <= 2:
        indice = -1
        while not isinstance(indice, int) or indice < 0 or indice > 2:
            indice = int(input("indice ? 0: Only go to standard pose, 1: Do Simple go to init, 2: Do control trajectory) : "))
    else:
        indice = int(sys.argv[2])
    execute_joint_trajectory_client(gripper_time, indice)
