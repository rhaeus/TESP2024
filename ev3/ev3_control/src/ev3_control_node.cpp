#include "ros/ros.h"

#include "ev3_control/WheelController.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ev3_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  WheelController wheel_controller(nh, nh_priv);
  ROS_INFO("ev3_control_node ready");
  ros::spin();

  return 0;
}
