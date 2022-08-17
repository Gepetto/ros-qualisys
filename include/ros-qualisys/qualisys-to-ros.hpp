#ifndef ROS_QUALISYS_QUALISYS_TO_ROS_HPP
#define ROS_QUALISYS_QUALISYS_TO_ROS_HPP

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "RTPacket.h"
#include "RTProtocol.h"

/**
 * @brief Global namespace of the package.
 */
namespace ros_qualisys {

/**
 * @brief This class is a ROS node allowing us to parametrize the connection
 * to a Qualisys system and export to /tf and /tf_static the streamed objects.
 *
 * We use rosparams in order to parametrize this nodes behavior.
 * In order to fin out the values of these parameters please refer to your
 * Qualisys system documentation.
 *
 * A quick example of YAML file for the parametrization.
 *
 * @code{.unparsed}
 *
 * ros_qualisys:
 *   server_address: "127.0.0.1" # This is the ip where the Qualisys system is
 * streaming. server_base_port: 22222 # This is the machine port where Qualisys
 * system is streaming. server_udp_port: 6734 # This is the udp port.
 *   major_version: 1 # Version of the SDK. (to be checked)
 *   minor_version: 20 # Version of the SDK. (to be checked)
 *   frame_rate: 1000 # Maximum rate at which the data are read and broadcasted
 * to ROS. fixed_frame_id: "mocap" # Name of the frame in which the Qualisys
 * object are expressed in.
 *
 * @endcode
 *
 * A practical example is available for a real setup in LAAS-CNRS in
 * ros-qualisys/config/bauzil-qualisys.yaml file.
 */
class QualisysToRos {
 public:
  /**
   * @brief Construct a new Qualisys To Ros object, this a simple default
   * constructor.
   */
  QualisysToRos();

  /**
   * @brief Destroy the Qualisys To Ros object.
   */
  ~QualisysToRos();

  /**
   * @brief Set the Node Handle object.
   *
   * @param nh pointer to a node handle.
   */
  void setNodeHandle(ros::NodeHandle& nh);

  /**
   * @brief Initialize the object by:
   *  - getting the ROS params.
   *  - starting the connection to the Qualisys system using the above params.
   *
   * @return true in case of success,
   * @return false otherwise
   */
  bool initialize();

  /**
   * @brief Acquire the data from the Qualisys stream and send it back to the
   * /tf or /tf_static topic.
   */
  void run();

  /**
   * @brief Properly shutdown the connection with the Qualisys system.
   */
  void terminate();

 private:
  /**
   * @brief Connect to the Qualisys system and check if the connection went
   * well. This method is used in the initialization.
   *
   * @return true in case of success.
   * @return false otherwise.
   */
  bool connect();

 private:
  /// @brief ROS node handle to fetch the ROS params.
  ros::NodeHandle* node_handle_;

  /// @brief Qualisys communication object.
  CRTProtocol crt_protocol_;

  // Server parameters.
  std::string server_address_;
  unsigned short base_port_;
  unsigned short udp_port_;
  int minor_version_;
  int major_version_;
  bool big_endian_;

  // Maximum rate at which application is running, can be less.
  int frame_rate_;

  // Publisher to the /tf ROS topic.
  bool publish_tf_;

  // Name of the fix frame in which the object are expressed in. This is the
  // mocap frame.
  std::string fixed_frame_id_;

  // Sanity check variables
  bool data_available_ = false;

  // Qualisys SDK acquiring data objects.
  CRTPacket::EPacketType packet_type_;
  CRTPacket* rt_packet_;
  unsigned int body_count_;
  const char* body_name_;
  float px_;
  float py_;
  float pz_;
  std::array<float, 9> rotation_matrix_;

  // ROS interface.
  std::shared_ptr<ros::Rate> rate_;
  tf2::Matrix3x3 ros_rotation_matrix_;
  tf2::Quaternion ros_quaternion_;
  geometry_msgs::TransformStamped ros_transform_;
  tf2_ros::TransformBroadcaster publisher_;
};

}  // namespace ros_qualisys

#endif  // ROS_QUALISYS_QUALISYS_TO_ROS_HPP

/** @example ros-qualisys.cpp
 * This is the actual usage of the QualisysToRos class in a ROS node.
 * The Node is defining the namespace so that if the node is launched
 * in a roslaunch file then the namespace will be the "name" of the node.
 * This way, in order to define the parameters in a yaml file, one need to
 * gives the same namespace as the node name.
 *
 * @see ros-qualisys/config/bauzil-qualisys.yaml
 * @see ros-qualisys/launch/qualisys_bauzil_bringup.launch
 */
