#! /usr/bin/env python3

import rospy
import sys
from panda_board.srv import MakeJointTrajectory
from std_msgs.msg import Float32, String



def make_joint_trajectory_client(dt):
    #message_pub = rospy.Publisher("/panda_board/situation", String, queue_size=1)
    rospy.wait_for_service("make_joint_traj")
    try:
        traj = rospy.ServiceProxy("make_joint_traj", MakeJointTrajectory)
        traj(dt)
        #message_pub.publish("End of making joint trajectory")
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

    
if __name__ == "__main__":
    if len(sys.argv) > 2:
        sys.exit("ERROR ! Too many arguments !")
    elif len(sys.argv) == 2:
        dt = float(sys.argv[1])
    else:
        dt = -1 
    while dt <= 0 or not isinstance(dt, float):
        dt = float(input("Enter delta time : "))
    make_joint_trajectory_client(dt)
