#include "ros/ros_translator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace dvl_a50::ros
{

double to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

geometry_msgs::msg::TwistWithCovarianceStamped translate_velocity_message(const dvl_a50::lib::VelocityMessage& msg)
{
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = rclcpp::Clock().now();
    twist_msg.header.frame_id = "dvl_frame";

    twist_msg.twist.twist.linear.x = msg.vx;
    twist_msg.twist.twist.linear.y = msg.vy;
    twist_msg.twist.twist.linear.z = msg.vz;

    twist_msg.twist.covariance.fill(0.0);

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) { 
            twist_msg.twist.covariance[i * 6 + j] = msg.covariance[i][j];
        }
    }

    return twist_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped translate_position_local_message(const dvl_a50::lib::PositionLocalMessage& msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = rclcpp::Clock().now();
    pose_msg.header.frame_id = "dvl_frame";

    pose_msg.pose.pose.position.x = msg.x;
    pose_msg.pose.pose.position.y = msg.y;
    pose_msg.pose.pose.position.z = msg.z;

    tf2::Quaternion q;
    q.setRPY(to_rad(msg.roll), to_rad(msg.pitch), to_rad(msg.yaw));
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_msg.pose.covariance.fill(0.0);


    return pose_msg;
}

} // namespace dvl_a50::ros
