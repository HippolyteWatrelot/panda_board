import rospy
import numpy as np
from panda_robot import PandaArm


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
