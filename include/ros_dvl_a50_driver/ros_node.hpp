#ifndef ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
#define ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP

#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/transducer_array.hpp>

#include "driver.hpp"

namespace dvl_a50::ros {

class DvlA50DriverNode : public rclcpp::Node {
   public:
    DvlA50DriverNode(const rclcpp::NodeOptions& options);

   private:
    void publish_twist(const dvl_a50::lib::VelocityMessage& msg);
    void publish_pose(const dvl_a50::lib::PositionLocalMessage& msg);
    void publish_altitude(const dvl_a50::lib::VelocityMessage& msg);
    void publish_transducer_array(const dvl_a50::lib::VelocityMessage& msg);

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_publisher_;
    rclcpp::Publisher<vortex_msgs::msg::DVLAltitude>::SharedPtr
        altitude_publisher_;
    rclcpp::Publisher<vortex_msgs::msg::TransducerArray>::SharedPtr
        transducer_array_publisher_;
    std::shared_ptr<dvl_a50::lib::DvlA50Driver> dvl_driver_;

    std::string frame_id_;

    bool publish_transducer_array_ = false;
};

}  // namespace dvl_a50::ros

#endif  // ROS_DVL_A50_DRIVER_ROS_ROS_NODE_HPP
