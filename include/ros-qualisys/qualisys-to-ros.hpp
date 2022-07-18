/**
 *
 */

#ifndef ROS_QUALISYS_QUALISYS_TO_ROS_HPP
#define ROS_QUALISYS_QUALISYS_TO_ROS_HPP

#include <ros/ros.h>

namespace ros_qualisys {

class QualisysToRos {
 public:
  QualisysToRos();

  ~QualisysToRos();

  void setNodeHandle(ros::NodeHandle& nh);

  void initialize();

 private:
  ros::NodeHandle* node_handle_;
};

}  // namespace ros_qualisys

#endif  // ROS_QUALISYS_QUALISYS_TO_ROS_HPP
