#! /usr/bin/env python3

import rospy
import numpy as np
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from panda_board.srv import MakeJointTrajectory

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

#from transform_utils import *



'''GLOBAL'''
EE_Trajectory = []
Xerrs = []
error = []
thetalist =   [0,               0, 0,              0, 0,             0,              0]
init_joint =  [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
final_joint = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
current_joint_states = JointState()

'''CONSTANT'''
#panda = URDF.load_from_parameter_server(verbose=False)
#panda = URDF.load_from_xml_file(os.path.abspath(os.getcwd()))
#tree = kdl_tree_from_urdf_model(panda)
#chain = tree.getChain("panda_link0", "panda_link8")

Blist = np.array([[0, 0, -1, 0, -0.088, 0],
                  [np.sqrt(2)/2, -np.sqrt(2)/2, 0, 0.593, 0, 0.088],
                  [0, 0, -1, 0, -0.088, 0],
                  [-np.sqrt(2)/2, np.sqrt(2)/2, 0, -0.277, 0, 0],                                               # [0, 1, 0, -0.277, 0, -0.0055]
                  [0, 0, -1, 0, -0.088, 0],
                  [-np.sqrt(2)/2, np.sqrt(2)/2, 0, 0.107, 0, -0.088],
                  [0, 0, 1, 0, 0, 0]]).T

Real_Blist = np.array([[0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [np.sqrt(2)/2, -np.sqrt(2)/2, 0, 0.593*np.cos(np.pi/4), 0.593*np.cos(np.pi/4), 0.088],
                       [0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [-np.sqrt(2)/2, np.sqrt(2)/2, 0, -0.277*np.cos(np.pi/4), -0.277*np.cos(np.pi/4), 0],
                       [0, 0, -1, 0.088*np.cos(np.pi/4), -0.088*np.cos(np.pi/4), 0],
                       [-np.sqrt(2)/2, np.sqrt(2)/2, 0, 0.107*np.cos(np.pi/4), 0.107*np.cos(np.pi/4), -0.088],
                       [0, 0, 1, 0, 0, 0]]).T
                       
Kp = np.diag(np.ones(6))                              
Ki = np.diag(15 * np.ones(6))
#Master_Tse = rospy.get_param("~Master_Tse")
#offset = [0, 0, 0, 0, 0, np.pi, np.pi/4]
#Home_thetalist = [0, 0, 0, 0, 0, 0, 0.785398163397]
Standard_thetalist = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]

#Home_Tse = np.array([[1,  0,  0, 0.088], 
#                     [0, -1,  0,     0], 
#                     [0,  0, -1, 0.926], 
#                     [0,   0,    0,  1]])
                     
Real_Home_Tse = np.array([[np.sqrt(2) / 2,  np.sqrt(2) / 2,  0, 0.088], 
                          [np.sqrt(2) / 2, -np.sqrt(2) / 2,  0,     0], 
                          [             0,               0, -1, 0.926], 
                          [             0,               0,  0,     1]])

Joints_Limits = np.array([[-2.8973, 2.8973, 2.1750], 
                          [-1.7628, 1.7628, 2.1750], 
                          [-2.8973, 2.8973, 2.1750], 
                          [-3.0718, -0.0698, 2.1750], 
                          [-2.8973, 2.8973, 2.6100], 
                          [-0.0175, 3.7525, 2.6100], 
                          [-2.8973, 2.8973, 2.6100]])

eomg = 0.001
ev = 0.0001




###################################################################################



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



##################################################################################




def joints_limits_satisfied(current_joints, previous_joints, dt):
    global Joints_limits
    for i in range(len(Joints_Limits)):
        if current_joints[i] < Joints_Limits[i, 0] or current_joints[i] > Joints_Limits[i, 1] or np.abs((current_joints[i] - previous_joints[i]) % np.pi) / dt > Joints_Limits[i, 2]:
            return False 
    return True



def reshape_to_transmatrix(vec):
    mat = np.zeros([4, 4])
    mat[-1, -1] = 1
    mat[:3, :3] = np.reshape(vec[:9], (3, 3))
    mat[:3, 3] = vec[9:]
    #pre_mat = FKinBody(mat, Blist, thetalist)
    return mat



def NextState(config, var_vec, dt=0.01, max_ang_speed=100):
    new_config = config
    for i in range(len(var_vec)):
        if abs(var_vec[i]) > max_ang_speed:
            print("ERROR : too large joint speed detected !")
            return 0
        new_config[i] += dt * var_vec[i]
    return new_config


def assert_joints_positions_rules(thetalist, epsilon=0.01):
    indices = []
    test = True
    for i in range(len(Joints_Limits)):
        if thetalist[i] < Joints_Limits[i, 0] + epsilon or thetalist[i] > Joints_Limits[i, 1] - epsilon:
            indices.append(i)
            test = False
    return test, indices
    
    
def right_angles(thetalist):
    for j in range(len(thetalist)):
        thetalist[j] = (thetalist[j] + np.pi) % (2 * np.pi) - np.pi
    return thetalist
    
    
def newton_raphson(Tse, init_joint_states):
    ref_thetalist = init_joint_states.position
    ref_Tse = FKinBody(Real_Home_Tse, Real_Blist, ref_thetalist)
    print("ref Tse:\n", ref_Tse)
    Vb_bracket = MatrixLog6(TransInv(ref_Tse) @ Tse)
    Vb = se3ToVec(Vb_bracket)
    aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)
    lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)
    thetalist = ref_thetalist
    print("first thetalist :", thetalist)
    T = standard_Tse
    while aem > eomg or lem > ev:
        k = 0
        Jb = JacobianBody(Real_Blist, thetalist)
        Jb_Pinv = np.linalg.pinv(Jb, 1e-3)
        thetalist += Jb_Pinv @ Vb
        thetalist = right_angles(thetalist)
        test, indices = assert_joints_positions_rules(thetalist)
        while not test:
            print(k)
            k += 1
            for i in range(len(indices)):
                Jb[:, i] = np.zeros(6)
            if not Jb.any():
                return thetalist, False
            Jb_Pinv = np.linalg.pinv(Jb, 1e-3)
            thetalist += Jb_Pinv @ Vb
            thetalist = right_angles(thetalist)
            test, indices = assert_joints_positions_rules(thetalist)
        print("NR thetalist: ", thetalist)
        T = FKinBody(Real_Home_Tse, Real_Blist, thetalist)
        Vb_bracket = MatrixLog6(TransInv(T) @ Tse)
        Vb = se3ToVec(Vb_bracket)
        aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)
        lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)
    return thetalist, True


def PID_Control(Tse, Tsed, TsedNext, Kp, Ki, Int_error, dt=0.01):

    Teed = TransInv(Tse) @ Tsed
    Mat_Xerr = MatrixLog6(Teed)
    Xerr = se3ToVec(Mat_Xerr)
    Mat_Vd = MatrixLog6(TransInv(Tsed) @ TsedNext) / dt
    Vd = se3ToVec(Mat_Vd)
    Ad_Teed = Adjoint(Teed)
    Int_error += Xerr * dt
    V = Ad_Teed @ Vd + Kp @ Xerr + Ki @ Int_error

    return V, Xerr, Int_error


def handle_joint_traj(req):
    global EE_Trajectory
    global thetalist
    global Xerrs
    global error
    global current_joint_states
    dt, inter = req.dt, req.inter
    thetalists = []
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    Tse = EE_Trajectory[0]
    Tsed = EE_Trajectory[0]
    joint_traj_point_pub = rospy.Publisher("/panda_board/candidate_joint_trajectory_point", JointTrajectoryPoint, queue_size=1)
    thetalist, test = newton_raphson(Tse, current_joint_states)
    if not test:
        print("Newton-Raphson did not converge !")
        return False
    else:
        print("Newton_Raphson Convergence succeeded !")
        print("Final NR thetalist: ", thetalist)
    for j in range(len(thetalist)):
        thetalist[j] = (thetalist[j] + np.pi) % (2 * np.pi) - np.pi
    Int_error = [0, 0, 0, 0, 0, 0]
    for i in range(1, len(EE_Trajectory)):
        joint_point = JointTrajectoryPoint()
        TsedNext = EE_Trajectory[i]
        Panda_Jacobian = JacobianBody(Real_Blist, thetalist)
        Panda_Jacobian_Pinv = np.linalg.pinv(Panda_Jacobian, 1e-3)
        V, Xerr, Int_error = PID_Control(Tse, Tsed, TsedNext, Kp, Ki, Int_error)
        Xerrs.append(Xerr)
        error_norm = np.mean(np.sqrt([np.sum(Xerr[i] ** 2) for i in range(len(Xerr))]))
        error.append(error_norm)
        u_theta = Panda_Jacobian_Pinv @ V
        new_thetalist = NextState(thetalist, u_theta, dt)
        if len(new_thetalist) == 7:
            for j in range(len(new_thetalist)):
                new_thetalist[j] = (new_thetalist[j] + np.pi) % (2 * np.pi) - np.pi
        if len(new_thetalist) != 7 or not joints_limits_satisfied(new_thetalist, thetalist, dt):
            print("ERROR ! Detected not respected joint limit rule in trajectory making !")
            return False
        thetalist = new_thetalist
        thetalists.append(thetalist)
        print("joints:\n", thetalist)
        if not np.array(thetalist).any():
            Joint_Trajectory = []
            et = -1
            while et not in [0, 1]:
                et = int(input("Erase Cartesian trajectory ? (0: no, 1: yes"))
            if et == 1:
                EE_Trajectory = []
            return False
        joint_point.positions = thetalists[i-1]
        joint_traj_point_pub.publish(joint_point)
        joint_trajectory.points.append(joint_point)
        Tse = FKinBody(Real_Home_Tse, Real_Blist, thetalist)
        Tsed = TsedNext
    print("Joint trajectory made !")
    EE_Trajectory = []
    return True
        


def handle_ee_traj(msg):
    global EE_Trajectory
    EE_Trajectory = []
    traj = msg.data
    length = int(len(traj) // 12)
    real_traj = np.reshape(traj, (length, 12))
    for i in range(length):
        EE_Trajectory.append(reshape_to_transmatrix(real_traj[i]))


def handle_current_joint_states(jointstate):
    global current_joint_states
    current_joint_states = jointstate


def handle_dt(msg):
    global dt
    dt = msg.data


def make_joint_trajectory_server():

    rospy.init_node("make_joint_traj")
    sub_current_joints_poses = rospy.Subscriber("/joint_states", JointState, handle_current_joint_states)
    '''Array of flattened Transform Matrices'''
    ee_traj_sub = rospy.Subscriber("/panda_board/candidate_ee_trajectory", Float32MultiArray, handle_ee_traj)
 
    s = rospy.Service("make_joint_traj", MakeJointTrajectory, handle_joint_traj)
    rospy.spin()


if __name__ == "__main__":
    make_joint_trajectory_server()
