#! /usr/bin/env python3

import rospy
import numpy as np
#from trajectory_types import *
from std_msgs.msg import Int16, Float32, Float32MultiArray, String
from panda_board.srv import MakeEEtrajectory



def RotInv(R):
    return np.array(R).T


def TransInv(T):
    R, p = TransToRp(T)
    Rt = RotInv(R)
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def MatrixLog3(R):
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not NearZero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return VecToso3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)


def MatrixLog6(T):
    R, p = TransToRp(T)
    omgmat = MatrixLog3(R)
    if np.array_equal(omgmat, np.zeros((3, 3))):
        return np.r_[np.c_[np.zeros((3, 3)),
                           [T[0][3], T[1][3], T[2][3]]],
                     [[0, 0, 0, 0]]]
    else:
        theta = np.arccos((np.trace(R) - 1) / 2.0)
        return np.r_[np.c_[omgmat,
                           np.dot(np.eye(3) - omgmat / 2.0 \
                           + (1.0 / theta - 1.0 / np.tan(theta / 2.0) / 2) \
                              * np.dot(omgmat,omgmat) / theta,[T[0][3],
                                                               T[1][3],
                                                               T[2][3]])],
                     [[0, 0, 0, 0]]]


def MatrixExp3(so3mat):
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)


def MatrixExp6(se3mat):
    se3mat = np.array(se3mat)
    omgtheta = so3ToVec(se3mat[0: 3, 0: 3])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        return np.r_[np.c_[MatrixExp3(se3mat[0: 3, 0: 3]),
                           np.dot(np.eye(3) * theta \
                                  + (1 - np.cos(theta)) * omgmat \
                                  + (theta - np.sin(theta)) \
                                    * np.dot(omgmat,omgmat),
                                  se3mat[0: 3, 3]) / theta],
                     [[0, 0, 0, 1]]]


def JacobianBody(Blist, thetalist):
    Jb = np.array(Blist).copy().astype(float)
    T = np.eye(4)
    for i in range(len(thetalist) - 2, -1, -1):
        T = np.dot(T,MatrixExp6(VecTose3(np.array(Blist)[:, i + 1] \
                                         * -thetalist[i + 1])))
        Jb[:, i] = np.dot(Adjoint(T), np.array(Blist)[:, i])
    return Jb


def FKinBody(M, Blist, thetalist):
    T = np.array(M)
    for i in range(len(thetalist)):
        T = np.dot(T, MatrixExp6(VecTose3(np.array(Blist)[:, i] \
                                          * thetalist[i])))
    return T


def Adjoint(T):
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                 np.c_[np.dot(VecToso3(p), R), R]]


def AxisAng3(expc3):
    return (Normalize(expc3), np.linalg.norm(expc3))


def Normalize(V):
    return V / np.linalg.norm(V)


def NearZero(z):
    return abs(z) < 1e-6


def TransToRp(T):
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]


def so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])


def VecToso3(omg):
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])


def VecTose3(V):
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]


def se3ToVec(se3mat):
    return np.r_[[se3mat[2][1], se3mat[0][2], se3mat[1][0]],
                 [se3mat[0][3], se3mat[1][3], se3mat[2][3]]]



###################################################################




def make_decoupled_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[:3, :3]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    print("nbr of points: ", int(100 * time))
    for i in range(int(100 * time)):
        s = (i + 1) / (time * 100)
        ps = pos_init + s * (pos_final - pos_init)
        for j in range(3):
            for k in range(3):
                traj.append(static_rot_matrix[j, k])
        for j in range(3):
            traj.append(ps[j])
    current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3] = ps[0], ps[1], ps[2]
    return traj, current_matrix


def make_decoupled_3_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[:3, :3]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    for i in range(int(100 * time)):
        T = int(100 * time)
        a2, a3 = 3 / (T ** 2), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        ps = pos_init + s * (pos_final - pos_init)
        for j in range(3):
            for k in range(3):
                traj.append(static_rot_matrix[j, k])
        for j in range(3):
            traj.append(ps[j])
    current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3] = ps[0], ps[1], ps[2]
    return traj, current_matrix


def make_decoupled_5_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    static_rot_matrix = current_matrix[:3, :3]
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    ps = []
    for i in range(int(100 * time)):
        T = int(100 * time)
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        ps = pos_init + s * (pos_final - pos_init)
        for j in range(3):
            for k in range(3):
                traj.append(static_rot_matrix[j, k])
        for j in range(3):
            traj.append(ps[j])
    current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3] = ps[0], ps[1], ps[2]
    return traj, current_matrix


def make_screw_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        s = (i + 1) / int(100 * time)
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        for j in range(3):
            for k in range(3):
                traj.append(Ts[j, k])
        for j in range(3):
            traj.append(Ts[j, 3])
    current_matrix = Ts
    return traj, current_matrix


def make_screw_Poly3_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        T = int(100 * time)
        a2, a3 = 3 / (T ** 3), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        for j in range(3):
            for k in range(3):
                traj.append(Ts[j, k])
        for j in range(3):
            traj.append(Ts[j, 3])   
    current_matrix = Ts
    return traj, current_matrix


def make_screw_Poly5_trajectory(current_matrix, final_matrix):
    traj = []
    pos_init = np.array([current_matrix[0, 3], current_matrix[1, 3], current_matrix[2, 3]])
    pos_final = np.array([final_matrix[0, 3], final_matrix[1, 3], final_matrix[2, 3]])
    dist = np.sqrt(np.sum(np.square(np.abs(pos_final - pos_init))))
    time = 10 * dist
    for i in range(int(100 * time)):
        T = int(100 * time)
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        Ts = current_matrix @ MatrixExp6(MatrixLog6(TransInv(current_matrix) @ final_matrix) * s)
        for j in range(3):
            for k in range(3):
                traj.append(Ts[j, k])
        for j in range(3):
            traj.append(Ts[j, 3])
    current_matrix = Ts
    return traj, current_matrix




##################################################################################################




mode = 0
init_matrix = np.diag(np.ones(4))
final_matrix = np.diag(np.ones(4))

p = {1: 'x', 2: 'y', 3: 'z'}
q = {1: 'w', 2: 'x', 3: 'y', 4: 'z'}

ee_traj_pub = rospy.Publisher("/panda_board/candidate_ee_trajectory", Float32MultiArray, queue_size=1)
message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)

Message = String()



#def normalize_quaternion(quaternion):
#    norm = np.sqrt(np.sum(np.square(quaternion)))
#    return quaternion / norm


def pose_to_transmatrix(p, q):
    return np.array([[2*(q[0]**2 + q[1]**2) - 1, 2*(q[1]*q[2] - q[0]*q[3]), 2*(q[1]*q[3] + q[0]*q[2]), p[0]],
                     [2*(q[1]*q[2] + q[0]*q[3]), 2*(q[0]**2 + q[2]**2) - 1, 2*(q[2]*q[3] - q[0]*q[1]), p[1]],
                     [2*(q[1]*q[3] - q[0]*q[2]), 2*(q[2]*q[3] + q[0]*q[1]), 2*(q[0]**2 + q[3]**2) - 1, p[2]],
                     [                        0,                         0,                         0,    1]])


#def handle_current_pose(msg):
#    global current_pose_matrix
#    pos = msg.pose.position
#    quat = msg.pose.orientation
#    current_pose_matrix = pose_to_transmatrix([pos.x, pos.y, pos.z], 
#                                              [quat.w, quat.x, quat.y, quat.z])

def handle_init_cartesian_pose(msg):
    global init_matrix
    tab = msg.tm
    for i in range(3):
        init_matrix[i] = tab[i]

def handle_traj_mode(msg):
    global mode 
    mode = int(msg.data)


def handle_EE_traj(req):
    global init_matrix
    global final_matrix
    global ee_traj_pub
    global message_pub
    
    ip, iq, fp, fq, mode = req.ip, req.iq, req.fp, req.fq, req.mode
    init_matrix = pose_to_transmatrix(ip, iq)
    final_matrix = pose_to_transmatrix(fp, fq)
    
    print("init matrix:\n", init_matrix)
    print()
    print("final matrix:\n", final_matrix)
    
    traj = []
    
    if mode == 1:
        traj, reached_matrix = make_decoupled_trajectory(init_matrix, final_matrix)
    elif mode == 2:
        traj, reached_matrix = make_decoupled_3_trajectory(init_matrix, final_matrix)
    elif mode == 3:
        traj, reached_matrix = make_decoupled_5_trajectory(init_matrix, final_matrix)
    elif mode == 4:
        traj, reached_matrix = make_screw_trajectory(init_matrix, final_matrix)
    elif mode == 5:
        traj, reached_matrix = make_screw_Poly3_trajectory(init_matrix, final_matrix)
    else:
        traj, reached_matrix = make_screw_Poly5_trajectory(init_matrix, final_matrix)
    trajectory = Float32MultiArray()
    trajectory.data = traj
    ee_traj_pub.publish(trajectory)
    print("TRAJECTORY:\n", trajectory)
    Message.data = "End Effector trajectory made !"
    message_pub.publish(Message)
    print("traj length: ", len(traj))
    return True


def make_ee_trajectory_server():

    rospy.init_node("make_ee_traj")
    mode_sub = rospy.Subscriber("/panda_board/traj_mode", Int16, handle_traj_mode)
    #ee_traj_sub = rospy.Subscriber("/panda_board/candidate_ee_trajectory", Float32MultiArray, None)
    #message_sub = rospy.Subscriber("/panda_board/situation", String, None)
    ee_traj_pub = rospy.Publisher("/panda_board/candidate_ee_trajectory", Float32MultiArray, queue_size=1)
    message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
 
    s = rospy.Service("make_ee_traj", MakeEEtrajectory, handle_EE_traj)
    rospy.spin()


if __name__ == "__main__":
    make_ee_trajectory_server()
