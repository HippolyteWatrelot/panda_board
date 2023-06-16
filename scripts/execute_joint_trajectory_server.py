#!/usr/bin/env python

import os
import time
import numpy as np
import random
import rospy
import actionlib
import sys
import time

from panda_board.srv import ExecuteJointTrajectory

from actionlib import SimpleActionClient

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult
)

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from std_msgs.msg import Float32, String, Int16

from geometry_msgs.msg import PoseStamped

from franka_gripper.msg import HomingActionGoal, GraspActionGoal, MoveActionGoal, StopActionGoal


'''CONSTANT'''
gripper_width = 0
homing_command = HomingActionGoal()
grasp_command = GraspActionGoal()
move_command = MoveActionGoal()
stop_command = StopActionGoal()
move_command.goal.speed = 1
grasp_command.goal.epsilon.inner = 0.3
grasp_command.goal.epsilon.outer = 0.3
grasp_command.goal.speed = 0.1
grasp_command.goal.force = 50
grasp_command.goal.width = 1

#action = rospy.resolve_name("~follow_joint_trajectory")
action = "/effort_joint_trajectory_controller/follow_joint_trajectory"
print("action: ", action)
print(type(action))
client = SimpleActionClient(action, FollowJointTrajectoryAction)

_timeout = 15
#param = rospy.resolve_name('~joint_pose')
#init_pose = rospy.get_param(param, None)
#if init_pose is None:
#    rospy.logerr('move_to_start: Could not find required parameter "' + param + '"')
#    sys.exit(1)

'''GLOBAL'''
Traj_points = []
command = 3
status = 0
joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
standard_joints = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
init_joints = [0, 0, 0, 0, 0, 0, 0]
queried_joint_states = JointTrajectoryPoint()
#for i in range(7):
#    queried_joint_state.position[i] = init_joints[i]
current_joint_states = JointState()


#Trajectory = JointTrajectory()



def ECT(req):

    global init_pose
    global init_joints
    global homing_command
    global grasp_command
    global command
    global status
    global current_joint_states
    global Traj_points
    
    grasp_pub = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal, queue_size=0)
    homing_pub = rospy.Publisher("/franka_gripper/homing/goal", HomingActionGoal, queue_size=0)
    message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
    
    init_Goal = FollowJointTrajectoryGoal()
    Goal = FollowJointTrajectoryGoal()
    
    action = "/effort_joint_trajectory_controller/follow_joint_trajectory"
    client = SimpleActionClient(action, FollowJointTrajectoryAction)


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



def handle_current_joint_states(jointstate):
    global current_joint_states
    current_joint_states = jointstate


#def handle_traj(traj):
#    global Trajectory
#    Trajectory = traj
    
def handle_traj_point(traj_point):
    global Traj_points
    Traj_points.append(traj_point)


def handle_init_pose(js):
    global init_pose
    queried_init_pose = dict(zip(js.name, js.position))
    for joint in queried_init_pose:
        init_pose[joint] = queried_init_pose[joint]


def handle_joints(msg):
    global init_joints
    init_joints = msg.data[0]


#def handle_command(msg):
#    global command = msg.data


def handle_status(msg):
    global status
    status = msg.data


def handle_queried_joints_states(joinstate):
    global queried_joint_states
    queried_joint_states = jointstate
 

def handle_ECT(req):
    f = ECT(req)
    return f


def execute_joint_trajectory_server():

    rospy.init_node("execute_joint_traj_server")

    #sub_current_pose = rospy.Subscriber("/static_cartesian_pose", PoseStamped, handle_current_pose)
    sub_current_joints_poses = rospy.Subscriber("/joint_states", JointState, handle_current_joint_states)
    #sub_queried_joint_pose = rospy.Subscriber("/panda_robot/queried_joint_states", JointTrajectoryPoint, handle_queried_joints_states)
    #sub_joint_trajectory = rospy.Subscriber("/panda_board/candidate_joint_trajectory", JointTrajectory, handle_traj)
    sub_joint_trajectory_point = rospy.Subscriber("/panda_board/candidate_joint_trajectory_point", JointTrajectoryPoint, handle_traj_point)
    #init_and_final_joint_sub = rospy.Subscriber("/panda_board/init_and_final_joints", Float32MultiArray, handle_joints)
    #command_sub = rospy.Subscriber("/panda_board/command_ind", Int16, handle_command)
    status_sub = rospy.Subscriber("/panda_board/status_ind", Int16, handle_status)

    s = rospy.Service("execute_joint_traj_service", ExecuteJointTrajectory, handle_ECT)

    rospy.sleep(1)
    rospy.spin()


if __name__ == "__main__":
    execute_joint_trajectory_server()
