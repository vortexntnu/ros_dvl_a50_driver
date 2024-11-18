#ifndef ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
#define ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP

#include "lib/tcp_client.hpp"
#include "lib/types.hpp"
#include "ros/ros_translator.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>

namespace dvl_a50::ros {

class DvlA50DriverNode : public rclcpp::Node {
public:
  DvlA50DriverNode();
  ~DvlA50DriverNode();

private:
  void publish_twist(const dvl_a50::lib::VelocityMessage &msg);
  void publish_pose(const dvl_a50::lib::PositionLocalMessage &msg);

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;
  std::shared_ptr<dvl_a50::lib::TcpClient> tcp_client_;
};

} // namespace dvl_a50::ros

#endif // ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
