/// @example

#include <ros/ros.h>

#include "ros-qualisys/qualisys-to-ros.hpp"

int main(int argc, char* argv[]) {
  // Classic definition of the node.
  ros::init(argc, argv, "ros-qualisys", ros::init_options::NoSigintHandler);

  // Here it says that the namespace of the Node can be overwritten by the
  // launcher
  ros::NodeHandle nh("~");

  // The node class.
  ros_qualisys::QualisysToRos q2r;
  // get the ROS node handle.
  q2r.setNodeHandle(nh);

  // Initialize the communication with the Qualisys system and get the ROS
  // params.
  if (!q2r.initialize()) {
    ROS_INFO("Initialization of the Qualisys driver failed!");
    return -1;
  }
  ROS_INFO("Successfully initialize Qualisys connection!");

  // Run the communication loop.
  while (ros::ok()) {
    q2r.run();
    ros::spinOnce();
  }

  // Properly shutdown the connexion to Qualisys
  ROS_INFO("Shutting down");
  q2r.terminate();

  // Properly shutting down the ROS node.
  ros::shutdown();
  return 0;
}
