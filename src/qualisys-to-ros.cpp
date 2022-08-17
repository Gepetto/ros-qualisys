#include "ros-qualisys/qualisys-to-ros.hpp"

#include <cmath>

namespace ros_qualisys {

QualisysToRos::QualisysToRos() {}

QualisysToRos::~QualisysToRos() {}

void QualisysToRos::setNodeHandle(ros::NodeHandle& nh) {
  node_handle_ = std::addressof(nh);
}

bool QualisysToRos::initialize() {
  assert(node_handle_ && "Wrong initialization of the node handle.");
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options

  int tmp_base_port(0);
  int tmp_udp_port(0);
  node_handle_->param("server_address", server_address_,
                      std::string("127.0.0.1"));
  node_handle_->param("server_base_port", tmp_base_port, 22222);
  node_handle_->param("server_udp_port", tmp_udp_port, 6734);
  node_handle_->param("major_version", major_version_, 1);
  node_handle_->param("minor_version", minor_version_, 20);
  node_handle_->param("frame_rate", frame_rate_, 100);
  node_handle_->param("fixed_frame_id", fixed_frame_id_, std::string("mocap"));
  base_port_ = static_cast<unsigned short>(tmp_base_port);
  udp_port_ = static_cast<unsigned short>(tmp_udp_port);

  rate_ = std::make_shared<ros::Rate>(frame_rate_);

  // sanity variables
  data_available_ = false;

  // Connecting to the server
  ROS_INFO_STREAM("QualisysToRos::initialize(): Connecting to the Qualisys "
                  << "Motion Tracking system specified at: " << server_address_
                  << ":" << base_port_);
  connect();
  return true;
}

bool QualisysToRos::connect() {
  data_available_ = false;

  // connect properly
  int failure = 0;
  bool success = false;
  while (failure < 20 && ros::ok() && !success) {
    ROS_INFO_STREAM("QualisysToRos::connect(): Try to connect");
    if (!crt_protocol_.Connected()) {
      if (!crt_protocol_.Connect(server_address_.c_str(), base_port_,
                                 &udp_port_, major_version_, minor_version_,
                                 big_endian_)) {
        ROS_INFO_STREAM(
            "QualisysToRos::connect(): Error while connecting: %s\n\n"
            << crt_protocol_.GetErrorString());
        ++failure;
        sleep(1);
        continue;
      }
    }
    ROS_INFO_STREAM("QualisysToRos::connect(): Are data available?");
    if (!data_available_) {
      if (!crt_protocol_.Read6DOFSettings(data_available_)) {
        ROS_INFO_STREAM("crt_protocol_.Read6DOFSettings: "
                        << crt_protocol_.GetErrorString());
        ++failure;
        sleep(1);
        continue;
      }
    }
    ROS_INFO_STREAM("QualisysToRos::connect(): Are data streamed");
    if (!crt_protocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, 0, NULL,
                                    CRTProtocol::cComponent6d)) {
      ROS_INFO_STREAM(
          "crt_protocol_.StreamFrames: " << crt_protocol_.GetErrorString());
      ++failure;
      sleep(1);
      continue;
    }
    success = true;
  }
  ROS_INFO_STREAM("QualisysToRos::connect(): Connected successfully");
  return failure >= 20 && success;
}

void QualisysToRos::run() {
  // here again?
  // connect();

  rt_packet_ = crt_protocol_.GetRTPacket();
  body_count_ = rt_packet_->Get6DOFBodyCount();
  ROS_INFO_THROTTLE(1, "Number of bodies found :%d", body_count_);

  if (crt_protocol_.Receive(packet_type_, true) ==
      CNetwork::ResponseType::success) {
    switch (packet_type_) {
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "QualisysToRos::run(): Error when streaming frames: "
                   << crt_protocol_.GetRTPacket()->GetErrorString());
        break;

      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data");
        break;

      case CRTPacket::PacketData:
        if (body_count_ <= 0) {
          ROS_WARN_THROTTLE(1, "QualisysToRos::run(): No Bodies Found");
        } else {
          for (unsigned int i = 0; i < body_count_; i++) {
            if (rt_packet_->Get6DOFBody(i, px_, py_, pz_,
                                        rotation_matrix_.data())) {
              if (std::isfinite(px_) && std::isfinite(px_) &&
                  std::isfinite(px_) && std::isfinite(rotation_matrix_[0]) &&
                  std::isfinite(rotation_matrix_[1]) &&
                  std::isfinite(rotation_matrix_[2]) &&
                  std::isfinite(rotation_matrix_[3]) &&
                  std::isfinite(rotation_matrix_[4]) &&
                  std::isfinite(rotation_matrix_[5]) &&
                  std::isfinite(rotation_matrix_[6]) &&
                  std::isfinite(rotation_matrix_[7]) &&
                  std::isfinite(rotation_matrix_[8])) {
                body_name_ = crt_protocol_.Get6DOFBodyName(i);
                ros_rotation_matrix_.setValue(
                    rotation_matrix_[0], rotation_matrix_[1],
                    rotation_matrix_[2], rotation_matrix_[3],
                    rotation_matrix_[4], rotation_matrix_[5],
                    rotation_matrix_[6], rotation_matrix_[7],
                    rotation_matrix_[8]);
                ros_rotation_matrix_.getRotation(ros_quaternion_);
                ros_quaternion_.normalize();
                // There a change of convention between ROS and Qualisys
                ros_quaternion_[3] = -ros_quaternion_[3];
                ros_quaternion_.normalize();
                ros_transform_.transform.translation.x = px_ / 1000.0;
                ros_transform_.transform.translation.y = py_ / 1000.0;
                ros_transform_.transform.translation.z = pz_ / 1000.0;
                ros_transform_.transform.rotation.x = ros_quaternion_.x();
                ros_transform_.transform.rotation.y = ros_quaternion_.y();
                ros_transform_.transform.rotation.z = ros_quaternion_.z();
                ros_transform_.transform.rotation.w = ros_quaternion_.w();
                ros_transform_.header.frame_id = fixed_frame_id_;
                ros_transform_.header.stamp = ros::Time::now();
                ros_transform_.child_frame_id = body_name_;
                publisher_.sendTransform(ros_transform_);
              }
            }
          }
        }

        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");
    }
  }
  rate_->sleep();
}

void QualisysToRos::terminate() {
  ROS_INFO("QualisysToRos::disconnect(): Shutting down the communication.");
  crt_protocol_.StreamFramesStop();
  crt_protocol_.Disconnect();
}

}  // namespace ros_qualisys
