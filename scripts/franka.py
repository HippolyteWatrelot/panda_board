#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JoinState
from geomtry_msgs.msg import Pose, PoseStamped, WrenchStamped
from std_msgs.msg import Float32, Float32MultiArray
import math
import numpy as np
import quaternion
import dynamic_reconfigure.client
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal

class Panda():
    def __init__(self):
        super(Panda, self).__init__()

        self.pos_pub = rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.configuration_pub = rospy.Publisher('/equilibrium.configuration', Float32MultiArray, queue_size=0)

        self.grasp_pub = rospy.Publisher('/franka/gripper/grasp/goal', GraspActionGoal, queues_size=0)
        self.move_pub = rospy.Publisher('/franka/gripper/move/goal', MoveActionGoal, queues_size=0)
        self.homing_pub = rospy.Publisher('/franka/gripper/homing/goal', HomingActionGoal, queues_size=0)
        self.stop_pub = rospy.Publisher('/franka/gripper/stop/goal', StopActionGoal, queues_size=0)

        self.force_feedback = 0
        self.set_K = dynamic_reconfigure.client.Client('dynamic_reconfigure_client_param_node', config_callback=None)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        self.move_command = MoveActionGoal()
        self.grasp_command = GraspActionGoal()
        self.home_command = HomeActionGoal()
        self.stop_command = StopActionGoal()
        self.gripper_width = 0
        self.move_command.goal.speed = 1
        self.grasp_command.goal.epsilon.inner = 0.3
        self.grasp_command.goal.epsilon.outer = 0.3
        self.grasp_command.goal.force = 50
        self.grasp_command.goal.width = 1

        rospy.sleep(1)


    def ee_pos_callback(self, msg):
        self.pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.y])
        self.ori = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

    def move_gripper(self, width):
        self.move_command.goal.width = width
        self.move_pub.publish(self.move_command)

    def grasp_gripper(self, width):
        self.grasp_command.goal.width = width
        self.grasp_pub.publish(self.grasp_command)

    def home(self):
        pos_array = [0.2, 0, 0.35]
        quat = np.quaternion(0, 1, 0, 0)
        goal = array_quat_2_pose(pos_array, quat)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.mow() 
        self.go_to_pose(goal)
        self.set_configuration(ns_msg)
        self.set_K.update_configuration(f'{str(self.name)}_nullspace_stiffness':10)

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration(f'{str(self.name)}_nullspace_stiffness':0)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)

    def joint_states_callback(self, msg):
        self.joint = msg.position[7:]
        self.gripper_width = msg.position[7] + msg.position[8]

    def force_feedback_callback(self, feedback):
        self.force = feedback.wrench.force
        self.force_feedback = np.linalg.norm(np.array([self.force.x, self.force.y, self.force.z]))

    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns):
        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns})

    def set_stiffness_key(self):
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)

    def set_configuration(self, joint):
        joint_description = Float32MultiArray()
        joint_description.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_description)

    def go_to_pose(self, goal, interp_dist=0.001, interp_dist_pol=0.001):         # planar gripper is quaternion (0, 1, 0, 0)
        r = rospy.Rate(100)
        start = self.pos
        start_ori = self.ori
        goal_array = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])

        dist = np.sqrt([(goal_array[0] - start[0]) ** 2 + (goal_array[1] - start[1]) ** 2 + (goal_array[2] - start[2]) ** 2])
        step_num_lin = math.floor(dist / interp_dist)

        qstart = np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        qgoal = np.quaternion(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z)

        scalar_prod = qstart.x * qgoal.x + qstart.y * qgoal.y + qstart.z * qgoal.z + qstart.w * qgoal.w

        if scalar_prod < 0:
            qstart.x = -qstart.x
            qstart.y = -qstart.y
            qstart.z = -qstart.z
            qstart.w = -qstart.w

        scalar_prod = qstart.x * qgoal.x + qstart.y * qgoal.y + qstart.z * qgoal.z + qstart.w * qgoal.w
        theta = np.arccos(np.abs(scalar_prod))
        print(theta)

        step_num_polar = math.floor(theta / interp_dist_polar)

        step_num = np.max([step_num_polar, step_num_lin])

        x = np.linspace(start[0], goal.pose.position.x, step_num)
        y = np.linspace(start[1], goal.pose.position.y, step_num)
        z = np.linspace(start[2], goal.pose.position.z, step_num)

        goal = PoseStamped()

        for i in range(step_num):
            quat = np.slerp_vectorized(qstart, qgoal, (i+1)/step_num)
            pos_array = np.array([x[i], y[i], z[i]])
            _goal = array_quat_2_pose(pos_array, quat)
            self.goal_pub.publish(_goal)
            r.sleep()
        rospy.sleep(0.2)



