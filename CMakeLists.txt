cmake_minimum_required(VERSION 2.8.3)
project(panda_board)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(cmake_modules)
IF(cmake_modules_FOUND)
  find_package(Eigen3 REQUIRED)
ELSE()
  find_package(Eigen REQUIRED)
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
ENDIF()
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  franka_hw
  franka_gripper
  trajectory_msgs  
  message_generation
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  realtime_tools
  hardware_interface
  joint_limits_interface
)

add_service_files(
  FILES  
  MakeEEtrajectory.srv 
  MakeJointTrajectory.srv
  ExecuteJointTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#find_package(cd_dynamics REQUIRED)
#find_package(dream_utils REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_board
  CATKIN_DEPENDS 
      std_msgs
      controller_interface
      dynamic_reconfigure
      eigen_conversions
      franka_hw
      franka_gripper
      geometry_msgs
      hardware_interface
      joint_limits_interface
      message_runtime
      realtime_tools
      roscpp
  DEPENDS Franka
)

catkin_install_python(PROGRAMS scripts/make_ee_trajectory_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/make_ee_trajectory_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/make_joint_trajectory_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/make_joint_trajectory_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/execute_joint_trajectory_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/execute_joint_trajectory_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/trajectory_types.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/transform_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


find_package(moveit_ros_planning_interface)

include_directories(
  .
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

add_executable( panda_board_node
  src/panda_board_node.cpp
  src/panda_board.cpp
)

target_link_libraries( panda_board_node
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

#add_executable(camera_handler src/camera_handler.cpp)
#target_link_libraries(camera_handler yaml-cpp ${catkin_LIBRARIES})

#add_executable(optitrack_handler_grasp src/optitrack_handler_grasp.cpp)
#target_link_libraries(optitrack_handler_grasp yaml-cpp ${catkin_LIBRARIES})
