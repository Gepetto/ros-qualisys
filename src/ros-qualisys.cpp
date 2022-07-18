/**
 *
 */

#include <ros/ros.h>

#include "ros-qualisys/qualisys-to-ros.hpp"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "my_node_name", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  ros_qualisys::QualisysToRos q2r;
  q2r.setNodeHandle(nh);
  q2r.initialize();

  ros::spin();
  return 0;
}
