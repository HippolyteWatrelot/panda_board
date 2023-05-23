#! /usr/bin/env python3

import numpy as np
import rospy
import sys
from panda_board.srv import MakeEEtrajectory
from std_msgs.msg import Int16, Float32, Float32MultiArray, String


default_delta_t = 0.01
default_init_position    = [0.2, 0, 0.4]
default_init_quaternion  = [1, 0, 0,  0]
default_final_position   = [0.4, 0, 0.4]
default_final_quaternion = [1, 0, 0,  0]

Message = String()

p = {1: 'x', 2: 'y', 3: 'z'}
q = {1: 'w', 2: 'x', 3: 'y', 4: 'z'}


def string_to_array(s, identity):
    n = 0
    try:
        if len(s) == 1:
            t = int(s)
        else:
            assert identity in ["init_position", "init_quaternion", "final_position", "final_quaternion"]
            if identity in ["init_position", "final_position"]:
                n = 3
            else:
                n = 4
            broken_s = s.split(',')
            assert broken_s[0][0] == '['
            assert broken_s[-1][-1] == ']'
            assert len(broken_s) == n
            t = []
            t.append(float(broken_s[0].split('[')[1]))
            for i in range(len(broken_s) - 2):
                t.append(float(broken_s[i+1]))
            t.append(float(broken_s[-1].split(']')[0]))
    except:
        sys.exit("Impossible to unstring %s ! Must be in bad format"%identity)
    return t

def normalize_quaternion(quaternion, mode):
    if mode == 'c':
        return [0, 0, 0, 0]
    norm = np.sqrt(np.sum(np.square(quaternion)))
    return quaternion / norm

def quaternion_to_rotmatrix(q):
    """"w x y z"""
    assert len(q) == 4
    return np.array([[2*(q[0]**2 + q[1]**2) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2])],
                     [2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]**2 + q[2]**2) - 1, 2*(q[2]*q[3] - q[0]*q[1])],
                     [2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]**2 + q[3]**2) - 1]])

def make_ee_trajectory_client(init_position, init_quaternion, final_position, final_quaternion, traj_mode):
    rospy.wait_for_service("make_ee_traj")
    try:
        traj = rospy.ServiceProxy("make_ee_traj", MakeEEtrajectory)
        traj(init_position, init_quaternion, final_position, final_quaternion, traj_mode)
        #message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
        #Message.data = "End of making end effector traj"
        #message_pub.publish(Message)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

    
if __name__ == "__main__":

    if len(sys.argv) == 2 or len(sys.argv) >= 6:
        sys.exit("bad arguments number")

    init_transmatrix = np.zeros([4, 4])
    final_transmatrix = np.zeros([4, 4])

    traj_mode = '0'
    while traj_mode not in ['1', '2', '3', '4', '5', '6']:
        traj_mode = input("Choose trajectory mode: \n "
                          "1 : Decoupled mode \n "
                          "2 : Decoupled_Poly3_mode \n "
                          "3 : Decoupled_Poly5_mode \n "
                          "4 : Screw_mode \n "
                          "5 : Screw_Poly3_mode \n "
                          "6 : Screw_Poly5_mode \n")
    traj_mode = int(traj_mode)          
    
    mode = 'j'
    if len(sys.argv) == 1:
        init_position = [0, 0, 0]
        init_quaternion = [1, 0, 0, 0]
        final_position = [0, 0, 0]
        final_quaternion = [1, 0, 0, 0]
        mode = 'c'
    elif sys.argv[1] == '1':
        init_position = []
        while len(init_position) != 3:
            init_position = []
            for i in range(3):
                value = float(input("enter init_position value %s : "%(p[i+1])))
                init_position.append(value)
    elif sys.argv[1] == '0':
        init_position = default_init_position
    else:
        init_position = string_to_array(sys.argv[1], "init_position")
    print("init_position: ", init_position)
    if (not isinstance(init_position, list) or len(init_position) != 3):
        sys.exit("Bad init_position argument")
    if sys.argv[2] == '1':
        init_quaternion = []
        while len(init_quaternion) != 4:
            init_quaternion = []
            for i in range(4):
                value = float(input("enter init_quaternion value %s : "%(q[i+1])))
                init_quaternion.append(value)
        init_quaternion = normalize_quaternion(init_quaternion, mode)
    elif sys.argv[2] == '0':
        init_quaternion = default_init_quaternion
    else:
        init_quaternion = string_to_array(sys.argv[2], "init_quaternion")
        init_quaternion = normalize_quaternion(init_quaternion, mode)
    print("init_quaternion: ", init_quaternion)
    if len(init_quaternion) != 4:
        sys.exit("Bad init_quaternion argument")
    init_transmatrix[:3, 3] = init_position
    if mode == 'j':
        init_transmatrix[:3, :3] = quaternion_to_rotmatrix(init_quaternion)
    else:
        init_transmatrix[:3, :3] = np.diag(np.ones([3, 3]))


    mode = 'j'
    if len(sys.argv) <= 3:
        final_position = [0, 0, 0]
        mode = 'c'
    elif sys.argv[3] == '1':
        final_position = []
        while len(final_position) != 3:
            final_position = []
            for i in range(3):
                value = float(input("enter final_position value %s : "%(p[i+1])))
                final_position.append(value)
    elif sys.argv[3] == '0':
        final_position = default_final_position
    else:
        final_position = string_to_array(sys.argv[3], "final_position")
    print("final_position: ", final_position)
    if len(final_position) != 3:
            sys.exit("Bad final position argument")
    ###
    if traj_mode <= 3 or len(sys.argv) == 4:
        final_quaternion = init_quaternion
    else:
        mode = 'j'
        if len(sys.argv) == 3:
            final_quaternion = [1, 0, 0, 0]
            mode = 'c'
        elif sys.argv[4] == '1':
            final_quaternion = []
            while len(final_quaternion) != 4:
                final_quaternion = []
                for i in range(4):
                    value = float(input("enter final_quaternion value %s : "%(q[i+1])))
                    final_quaternion.append(value)
            final_quaternion = normalize_quaternion(final_quaternion, mode)
        elif sys.argv[4] == '0':
            final_quaternion = default_final_quaternion
        else:
            final_quaternion = string_to_array(sys.argv[4], "final_quaternion")
            final_quaternion = normalize_quaternion(final_quaternion, mode)
        print("final_quaternion: ", final_quaternion)
        if len(final_quaternion) != 4:
            sys.exit("Bad final quaternion argument")
    final_transmatrix[:3, 3] = final_position
    if mode == 'j':
        final_transmatrix[:3, :3] = quaternion_to_rotmatrix(final_quaternion)
    else:
        final_transmatrix[:3, :3] = np.diag(np.ones([3, 3]))
        
    print("final_quaternion: ", final_quaternion)
    
    print("init_transmatrix:\n", init_transmatrix)
    print()
    print("final_transmatrix:\n", final_transmatrix)
    
    #init_transmatrix = [list(init_transmatrix[r]) for r in range(init_transmatrix.shape[0] - 1)]
    #final_transmatrix = [list(final_transmatrix[r]) for r in range(final_transmatrix.shape[0] - 1)]
    #make_ee_trajectory_client(init_transmatrix, final_transmatrix, traj_mode)
    make_ee_trajectory_client(init_position, init_quaternion, final_position, final_quaternion, traj_mode)
