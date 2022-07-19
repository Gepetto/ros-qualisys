/**
 *
 */

#ifndef ROS_QUALISYS_QUALISYS_TO_ROS_HPP
#define ROS_QUALISYS_QUALISYS_TO_ROS_HPP

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "RTPacket.h"
#include "RTProtocol.h"

namespace ros_qualisys {

class QualisysToRos {
 public:
  QualisysToRos();

  ~QualisysToRos();

  void setNodeHandle(ros::NodeHandle& nh);

  bool initialize();

  void run();

  void terminate();

 private:
  bool connect();

 private:
  ros::NodeHandle* node_handle_;

  // Qualisys objects:
  CRTProtocol crt_protocol_;

  // params
  std::string server_address_;
  unsigned short base_port_;
  unsigned short udp_port_;
  int minor_version_;
  int major_version_;
  bool big_endian_;
  int frame_rate_;
  double max_accel_;
  bool publish_tf_;
  std::string fixed_frame_id_;

  // sanity check variables
  bool data_available_ = false;

  // acquiring data
  CRTPacket::EPacketType packet_type_;
  CRTPacket* rt_packet_;
  unsigned int body_count_;
  const char* body_name_;
  float px_;
  float py_;
  float pz_;
  std::array<float, 9> rotation_matrix_;

  // ROS interface
  std::shared_ptr<ros::Rate> rate_;
  tf2::Matrix3x3 ros_rotation_matrix_;
  tf2::Quaternion ros_quaternion_;
  geometry_msgs::TransformStamped ros_transform_;
  tf2_ros::TransformBroadcaster publisher_;
};

}  // namespace ros_qualisys

#endif  // ROS_QUALISYS_QUALISYS_TO_ROS_HPP
