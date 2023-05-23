#ifndef PANDA_BOARD_H
#define PANDA_BOARD_H

#include "panda_board/common.h"

#include <franka/gripper.h>
#include <franka/robot_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>

#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <Eigen/Core>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>


struct SParam {

  double timestep;
  bool use_gripper;
  double gripper_gripping_early_time;
  double grasp_width;

  std::vector<double > init_pose_arm;
};


typedef boost::shared_ptr<ros::AsyncSpinner> AsyncSpinnerPtr;

class PandaBoard
{
public:
  Eigen::Vector3d mBarPosition;
  Eigen::Vector4d mBarOrientation;

  Eigen::VectorXd mJointPos, mJointVel;

  PandaBoard();

protected:
  AsyncSpinnerPtr mAsyncSpinner;

  ros::NodeHandle _nh;
  
  ros::Subscriber _joint_state_sub;
  ros::Subscriber _cart_state_sub;
  
  ros::Subscriber _move_gripper_width_sub;
  ros::Subscriber _grasp_gripper_width_sub;
  ros::Subscriber _situation_sub;
  
  ros::Publisher _homing_gripper_pub;
  ros::Publisher _move_gripper_pub;
  ros::Publisher _grasp_gripper_pub;
  ros::Publisher _stop_gripper_pub;

  ros::Subscriber _command_sub;  
  ros::Publisher  _status_pub;

  //ros::Publisher _rviz_bar_pose_pub;

  float _move_gripper_width;
  float _grasp_gripper_width;
  
  sensor_msgs::JointState _current_joints_state;
  Eigen::Vector3d _current_ee_position;
  Eigen::Vector4d _current_ee_quaternion;

  SParam _param;

  franka_gripper::MoveActionGoal _move_command;
  franka_gripper::GraspActionGoal _grasp_command;
  franka_gripper::HomingActionGoal _homing_command;
  franka_gripper::StopActionGoal _stop_command;

  PANDA_BOARD::ENUM_PANDA_STATUS _status;
  PANDA_BOARD::ENUM_PANDA_ACTION _action_status;

  void joint_state_handler(sensor_msgs::JointState jointstate);

  void cart_state_handler(geometry_msgs::PoseStamped cartesian_pose);

  //void barpose_handler(const geometry_msgs::PoseStamped msg);

  void situation_handler(std_msgs::String msg);
  
  bool move_gripper_width_handler(std_msgs::Float32 value);
  
  bool grasp_gripper_width_handler(std_msgs::Float32 value);

  void single_command_handler(std_msgs::Int16 command_msg);

  void read_param();

  void panda_status(PANDA_BOARD::ENUM_PANDA_STATUS status);

};


#endif // PANDA_BOARD_H
