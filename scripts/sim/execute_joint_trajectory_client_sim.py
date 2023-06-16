#! /usr/bin/env python3

import rospy
import numpy as np
import sys

from panda_board.srv import ExecuteJointTrajectorySim

from std_msgs.msg import Float32, Int16, String


def execute_joint_trajectory_client_sim(gripper_time):
    rospy.wait_for_service("execute_joint_traj_service")
    try:
        traj = rospy.ServiceProxy("execute_joint_traj_service_sim", ExecuteJointTrajectorySim)
        traj(gripper_time)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


if __name__ == "__main__":

    if len(sys.argv) == 1:
        gripper_time = -1
        test = False
        while not test or float(gripper_time) < 0:
            test = False
            gripper_time = input("time for gripper to close : ")
            try:
                gripper_time = float(gripper_time)
                test = True
            except:
                pass
    else:
        try:
            gripper_time = float(sys.argv[1])
            test = True
        except:
            test = False
        while not test:
            test = False
            try:
                gripper_time = float(sys.argv[1])
                test = True
            except:
                pass
    execute_joint_trajectory_client_sim(gripper_time)
