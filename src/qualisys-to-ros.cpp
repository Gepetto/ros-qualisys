/**
 *
 */

#include "ros-qualisys/qualisys-to-ros.hpp"

namespace ros_qualisys {

QualisysToRos::QualisysToRos() {}

QualisysToRos::~QualisysToRos() {}

void QualisysToRos::setNodeHandle(ros::NodeHandle& nh) {
  node_handle_ = std::addressof(nh);
}

void QualisysToRos::initialize() {}

}  // namespace ros_qualisys
