#ifndef ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
#define ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "driver.hpp"

namespace dvl_a50::ros {

class DvlA50DriverNode : public rclcpp::Node {
   public:
    DvlA50DriverNode();

   private:
    void publish_twist(const dvl_a50::lib::VelocityMessage& msg);
    void publish_pose(const dvl_a50::lib::PositionLocalMessage& msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_publisher_;
    std::shared_ptr<dvl_a50::lib::DvlA50Driver> dvl_driver_;

    std::string frame_id_;
};

}  // namespace dvl_a50::ros

#endif  // ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
