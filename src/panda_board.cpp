#include "panda_board/panda_board.h"
//#include <franka_ros/franka_gripper.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sys/stat.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_gripper/GraspActionGoal.h>
#include <franka_gripper/HomingActionGoal.h>
#include <franka_gripper/StopActionGoal.h>
#include <franka_gripper/MoveActionGoal.h>

ros::Time t_begin;
double t_begin_second;


void get_time_dir(std::string& filename)
{
    time_t rawtime;
    struct tm * timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    char filename_char[50];

    sprintf(filename_char, "%04d%02d%02d_%02d%02d%02d",
	    timeinfo->tm_year+1900, timeinfo->tm_mon, timeinfo->tm_mday,
	    timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    filename = std::string(filename_char);
}


PandaBoard::PandaBoard()
{
    read_param();

    _joint_state_sub = _nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &PandaBoard::joint_state_handler, this);

    _homing_gripper_pub = _nh.advertise<franka_gripper::HomingActionGoal>("/franka_gripper/homing/goal", true);
    _move_gripper_pub = _nh.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal", true);
    _grasp_gripper_pub = _nh.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", true);
    _stop_gripper_pub = _nh.advertise<franka_gripper::StopActionGoal>("/franka_gripper/stop/goal", true);

    _command_sub = _nh.subscribe<std_msgs::Int16>(PANDA_BOARD::topic_command, 1, &PandaBoard::single_command_handler, this);

    _status_pub  = _nh.advertise<std_msgs::Int16>(PANDA_BOARD::topic_status, true);

    //_rviz_board_barycenter_pose_pub  = _nh.advertise<geometry_msgs::PointStamped>(BAXTER_GRASPING::topic_rviz_table_pose, true);

    _status = PANDA_BOARD::PANDA_STATUS_IDLE;
    _action_status = PANDA_BOARD::ACTION_STATUS_IDLE;

    std::string date_dir;
    get_time_dir(date_dir);

    mAsyncSpinner = AsyncSpinnerPtr(new ros::AsyncSpinner(16));
    mAsyncSpinner->start();

    ROS_INFO("panda board manager started");

    
}

void PandaBoard::read_param()                               // roslaunch
{
    _nh.getParam("timestep", _param.timestep);

    //_nh.getParam("local_urdf_file", _param.local_urdf_file);

    //_nh.getParam("panda_description", _param.robot_description);

    _param.init_pose_arm.resize(7);
    _nh.param<double >("init_pos_arm_j0", _param.init_pose_arm[0], PANDA_BOARD::panda_arm_init_pos[0]);
    _nh.param<double >("init_pos_arm_j1", _param.init_pose_arm[1], PANDA_BOARD::panda_arm_init_pos[1]);
    _nh.param<double >("init_pos_arm_j2", _param.init_pose_arm[2], PANDA_BOARD::panda_arm_init_pos[2]);
    _nh.param<double >("init_pos_arm_j3", _param.init_pose_arm[3], PANDA_BOARD::panda_arm_init_pos[3]);
    _nh.param<double >("init_pos_arm_j4", _param.init_pose_arm[4], PANDA_BOARD::panda_arm_init_pos[4]);
    _nh.param<double >("init_pos_arm_j5", _param.init_pose_arm[5], PANDA_BOARD::panda_arm_init_pos[5]);
    _nh.param<double >("init_pos_arm_j6", _param.init_pose_arm[6], PANDA_BOARD::panda_arm_init_pos[6]);
}


void PandaBoard::joint_state_handler(sensor_msgs::JointState jointstate)
{
    for(unsigned int i = 0; i < PANDA_BOARD::panda_joints_names.size(); ++i) {
    int idx = distance(jointstate.name.begin(),
                       find(jointstate.name.begin(),
                            jointstate.name.end(),
                            PANDA_BOARD::panda_joints_names[i])
                       );
    if( (idx >=0) && (idx<jointstate.position.size())) mJointPos(i) = jointstate.position[idx];
    if( (idx >=0) && (idx<jointstate.velocity.size())) mJointVel(i) = jointstate.velocity[idx];
    _current_joints_state.position = jointstate.position;
    }
}



void PandaBoard::single_command_handler(const std_msgs::Int16 command_msg)
{
    int command = command_msg.data;

    switch(command) {
    case PANDA_BOARD::CMD_BOARD_DETECTION:
        panda_status(PANDA_BOARD::PANDA_STATUS_BOARD_DETECTION);
	break;
    case PANDA_BOARD::CMD_FEATURES_DETECTION:
	    panda_status(PANDA_BOARD::PANDA_STATUS_FEATURES_DETECTION);
	break;
    case PANDA_BOARD::CMD_MAKE_EE_TRAJECTORY:
        panda_status(PANDA_BOARD::PANDA_STATUS_MAKING_EE_TRAJECTORY);
        //_static_joint_pose_pub.publish(_current_joint_states, "/static_joint_poses", true);    // for traj making server
	    ros::Duration(1).sleep();
	    // then calling traj making client
	break;
    case PANDA_BOARD::CMD_MAKE_JOINT_TRAJECTORY:
        panda_status(PANDA_BOARD::PANDA_STATUS_MAKING_JOINT_TRAJECTORY);
	    ros::Duration(1).sleep();
	break;
    case PANDA_BOARD::CMD_GO_JOB:
        if (_status == PANDA_BOARD::PANDA_STATUS_MAKING_JOINT_TRAJECTORY) {
            panda_status(PANDA_BOARD::PANDA_STATUS_EXECUTING_JOINT_TRAJECTORY);	
        }
        else {
            panda_status(PANDA_BOARD::PANDA_STATUS_GOING_HOME);
        }
        break;
    case PANDA_BOARD::CMD_GO_HOME:
        panda_status(PANDA_BOARD::PANDA_STATUS_GOING_HOME);
        break;
    case PANDA_BOARD::CMD_GRIP_HOMING:
        panda_status(PANDA_BOARD::PANDA_STATUS_GRIPPER_HOMING);
        _homing_gripper_pub.publish(_homing_command);
        break;
    case PANDA_BOARD::CMD_GRIP_MOVE:
        panda_status(PANDA_BOARD::PANDA_STATUS_GRIPPER_MOVE);
        //_move_command.width = _move_gripper_width;
        _move_gripper_pub.publish(_move_command);
        break;
    case PANDA_BOARD::CMD_GRIP_GRASP:
        panda_status(PANDA_BOARD::PANDA_STATUS_GRIPPER_GRASP);
       // _grasp_command.width = _grasp_gripper_width;
        _grasp_gripper_pub.publish(_grasp_command);
        break;
    case PANDA_BOARD::CMD_GRIP_STOP:
        panda_status(PANDA_BOARD::PANDA_STATUS_GRIPPER_STOP);
        _stop_gripper_pub.publish(_stop_command);
        break;
    }
}


void PandaBoard::panda_status(PANDA_BOARD::ENUM_PANDA_STATUS status)
{
    std_msgs::Int16 msg;
    msg.data = status;
    _status_pub.publish(msg);
    _status = status;
}

