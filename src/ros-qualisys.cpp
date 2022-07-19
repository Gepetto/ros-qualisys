/**
 *
 */

#include <ros/ros.h>

#include "ros-qualisys/qualisys-to-ros.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ros-qualisys", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  ros_qualisys::QualisysToRos q2r;
  q2r.setNodeHandle(nh);

  if (!q2r.initialize()) {
    ROS_INFO("Initialization of the Qualisys driver failed!");
    return -1;
  }
  ROS_INFO("Successfully initialize Qualisys connection!");

  while (ros::ok()) {
    q2r.run();
    ros::spinOnce();
  }

  ROS_INFO("Shutting down");
  q2r.terminate();

  ros::shutdown();
  return 0;
}
