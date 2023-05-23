#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <string>

namespace  PANDA_BOARD {
  enum ENUM_PANDA_STATUS {
        PANDA_STATUS_IDLE=0,
        PANDA_STATUS_MAKING_EE_TRAJECTORY,
        PANDA_STATUS_MAKING_JOINT_TRAJECTORY,
        PANDA_STATUS_EXECUTING_JOINT_TRAJECTORY,
        PANDA_STATUS_GOING_HOME,
        PANDA_STATUS_GOING_TO_CONFIG,
        PANDA_STATUS_GRIPPER_HOMING,
        PANDA_STATUS_GRIPPER_STOP,
        PANDA_STATUS_GRIPPER_MOVE,
        PANDA_STATUS_GRIPPER_GRASP,
        PANDA_STATUS_START_POSITION,
        PANDA_STATUS_TO_BOX_CLOSING_1,
        PANDA_STATUS_TO_TBOX_CLOSING_2,
        PANDA_STATUS_BOARD_DETECTION,
        PANDA_STATUS_FEATURES_DETECTION,
        PANDA_STATUS_COMMAND_INVALID
        };

  enum ENUM_PANDA_COMMAND {
        CMD_INIT=0,
        CMD_SAY_HELLO,                       //1    
        CMD_GO_HOME,                         //2
        CMD_GO_JOB,                          //3
        CMD_MAKE_EE_TRAJECTORY,              //4
        CMD_MAKE_JOINT_TRAJECTORY,           //5
        CMD_EXECUTE_JOINT_TRAJECTORY,        //6                   
        CMD_BOARD_DETECTION,                 //7
	CMD_FEATURES_DETECTION,              //8 
        CMD_GRIP_MOVE,                       //9
        CMD_GRIP_HOMING,                     //10
        CMD_GRIP_STOP,                       //11
        CMD_GRIP_GRASP,                      //12
        CMD_JOINT_BAR_GRASP,                 //13
        CMD_CONTROL_BOX_CLOSING_1,           //14
        CMD_CONTROL_BOX_CLOSING_2,           //15
        CMD_ACTION_IDLE,                     //16
        CMD_SAVE                             //      
        };
        
  enum ENUM_PANDA_ACTION {
        ACTION_STATUS_IDLE,
        ACTION_TRAJECTORY,
        ACTION_GRASPING,
        ACTION_GRIP_HOMING,
        ACTION_GRIP_STOP,
        ACTION_GRIP_MOVE
        };

  const static std::string topic_command = "/panda_board/command_ind";
  const static std::string topic_status = "/panda_board/status_ind";

  const static std::string topic_board = "/panda_board/board_estimated_pose";
  const static std::string topic_rviz_board = "/panda_board/rviz_table_estimated_pose";

  const static std::string topic_trajectory_mode = "/panda_board/trajectory_mode";
  const static std::string topic_candidate_ee_trajectory = "/panda_board/candidate_ee_trajectory";
  const static std::string topic_candidate_joint_trajectory = "/panda_board/candidate_joint_trajectory";
  const static std::string topic_init_and_final_joint = "/panda_board/init_and_final_joints";
  const static std::string topic_board_pose = "/panda_board/board_pose";
  const static std::string topic_offset_cartesian_pose = "/offset_cartesian_pose";
  const static std::string topic_situation = "/panda_board/candidate_ee_trajectory";

  const static std::vector<std::string> panda_joints_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  const static std::vector<double > panda_arm_init_pos  = {0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397};
}

#endif // COMMON_H
