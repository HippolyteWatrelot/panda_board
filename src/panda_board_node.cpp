#include <ros/ros.h>
#include <panda_board/panda_board.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "panda_board");

  PandaBoard manager;

  ros::spin();

  return 0;
}

