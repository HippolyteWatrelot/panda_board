import rospy
import numpy as np
from panda_robot import PandaArm
import os
import time
import sys

from panda_board.srv import ExecuteJointTrajectorySim

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from sensor_msgs.msg import JointState

from franka_tools import JointTrajectoryActionClient

status = 0
joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
standard_joints = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
init_joints = [0, 0, 0, 0, 0, 0, 0]
current_joint_states = JointState()
Traj_points = []


def ECT(req):

    #global init_pose
    global init_joints
    global status
    global current_joint_states
    global Traj_point

    p = PandaArm()
    rospy.loginfo("Move to init pose: Waiting for '" + action + "' action to come up")
    gripper_time = req.gt
    #print("Type Trajectory: ", type(Trajectory.points))
    #print("Trajectory: ", Trajectory.points)
    print("Traj points: ", Traj_points)
    if gripper_time < 0.005:
        while type(gripper_time) != float or gripper_time < 0.005:
            gripper_time = float(input("set gripper time: "))
    init_joints = Traj_points[0].positions
    p.move_to_joint_position(init_joints)
   
    print("sleeping for 1 second...")
    rospy.sleep(1)
    '''Now specific trajectory'''
    rospy.loginfo("Straight trajectory: Waiting for '" + action + "' action to come up")
    traj_client = JointTrajectoryActionClient(joint_names)
    _timeout = 15
    rospy.sleep(3)
    for k in range(len(Traj_points)):
        traj_client.add_point(Traj_points[k].positions, k * 0.01)
    rospy.sleep(1)
    traj_client.start()
    traj_client.wait(gripper_time)###     
    ###client.wait_for_result(timeout=rospy.Duration(max(_timeout - gripper_time, 0)))###
    result = traj_client.result()
    Traj_points = []
    if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
        print("Full task successful")
        return True
    else:
        print("Specific trajectory failed")
        return False
    print("Move to init position failed")
    return False


def handle_traj_point(traj_point):
    global Traj_points
    Traj_points.append(traj_point)

def handle_status(msg):
    global status
    status = msg.data

def handle_current_joint_states_sim(jointstate):
    global current_joint_states
    current_joint_states = jointstate

    

def execute_joint_trajectory_server_sim():

    rospy.init_node("execute_joint_traj_server_sim")

    sub_current_joints_poses_sim = rospy.Subscriber("/panda_simulator/custom_franka_state_controller/joint_states", JointState, handle_current_joint_states_sim)
    sub_joint_trajectory_point = rospy.Subscriber("/panda_board/candidate_joint_trajectory_point", JointTrajectoryPoint, handle_traj_point)
    status_sub = rospy.Subscriber("/panda_board/status_ind", Int16, handle_status)

    s = rospy.Service("execute_joint_traj_service_sim", ExecuteJointTrajectorySim, handle_ECT)

    rospy.sleep(1)
    rospy.spin()


if __name__ == "__main__":
    execute_joint_trajectory_server_sim()
