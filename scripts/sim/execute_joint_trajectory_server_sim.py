import rospy
import numpy as np
from panda_robot import PandaArm
import os
import time
import sys

from panda_board.srv import ExecuteJointTrajectory

from sensor_msgs.msg import JointState


def ECT(req):

    global init_pose
    global init_joints
    global homing_command
    global grasp_command
    global command
    global status
    global current_joint_state
    global queried_joint_state
    global Traj_point

    rospy.loginfo("Move to init pose: Waiting for '" + action + "' action to come up")
    gripper_time = req.gt
    #print("Type Trajectory: ", type(Trajectory.points))
    #print("Trajectory: ", Trajectory.points)
    print("Traj points: ", Traj_points)
    if gripper_time < 0.005:
        while type(gripper_time) != float or gripper_time < 0.005:
            gripper_time = float(input("set gripper time: "))
    init_joints = Traj_points[0].positions
    print("first: ", current_joint_states.position)
    print("last: ", init_joints)
    max_movement = max(abs(init_joints[i] - current_joint_states.position[i]) for i in range(len(init_joints)))
    init_point = JointTrajectoryPoint()                                                                           # init_point represents query
    init_point.time_from_start = rospy.Duration.from_sec(max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5))
    #init_Goal.trajectory.joint_names, init_point.positions = [list(x) for x in zip(*init_pose.items())]
    init_Goal.trajectory.joint_names = [joint for joint in joint_names]
    #init_point.positions = queried_joint_state.positions
    init_point.positions = init_joints
    init_point.velocities = [0] * 7
    init_Goal.trajectory.points.append(init_point)
    init_Goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
    print("INIT GOAL")
    print(init_Goal.trajectory.joint_names)
    print(init_Goal.trajectory.points[0].positions)
    print(init_Goal.trajectory.points[0].velocities)
    rospy.loginfo('Sending trajectory Goal to move into queried initial config')
    client.send_goal_and_wait(init_Goal)
    init_result = client.get_result()
    
    indice = 2
    if init_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
        client = SimpleActionClient(action, FollowJointTrajectoryAction)
        if indice == 0:
            print("Homing traj Command successful !")
            message_pub.publish("Homing traj Command successful !")
            return True
        elif indice == 1:
            print("Success for init position !")
            return True
        elif len(Traj_points) == 0:
            print("Success for init position !")
            ros.loginfo("No specific trajectory to execute !")
            return False
        print("sleeping for 3 seconds...")
        rospy.sleep(3)
        '''Now specific trajectory'''
        rospy.loginfo("Straight trajectory: Waiting for '" + action + "' action to come up")
        _timeout = 15
        rospy.sleep(3)
        for k in range(len(Traj_points)):
            Traj_points[k].time_from_start = rospy.Duration(k * 0.01)
            Goal.trajectory.points.append(Traj_points[k])
        ###Goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)###
        Goal.trajectory.joint_names = joint_names
        ###homing_pub.publish(homing_command)###
        rospy.sleep(2)
        Goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)
        Goal.trajectory.header.stamp = rospy.Time.now()
        client.send_goal(Goal)                                            # <-----------------------------------------------------    TRAJECTORY
        client.wait_for_result(timeout=rospy.Duration(gripper_time))###     
        ###grasp_pub.publish(grasp_command)###
        ###client.wait_for_result(timeout=rospy.Duration(max(_timeout - gripper_time, 0)))###
        result = client.get_result()
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
