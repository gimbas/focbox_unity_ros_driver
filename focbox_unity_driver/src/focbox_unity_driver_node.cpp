#include <ros/ros.h>

#include "focbox_unity_driver/focbox_unity_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "focbox_unity_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  focbox_unity_driver::FocboxUnityDriver focbox_unity_driver(nh, private_nh);

  ros::spin();

  return 0;
}
