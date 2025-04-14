#ifndef ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP
#define ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP

#include "types.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <vortex_msgs/msg/dvl_altitude.hpp>
#include <vortex_msgs/msg/transducer_array.hpp>

namespace dvl_a50::ros {

geometry_msgs::msg::TwistWithCovarianceStamped velocity_message_to_twist(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string& frame_id);
geometry_msgs::msg::PoseWithCovarianceStamped position_local_to_pose(
    const dvl_a50::lib::PositionLocalMessage& msg,
    const std::string frame_id);
vortex_msgs::msg::DVLAltitude velocity_message_to_altitude(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string frame_id);
vortex_msgs::msg::TransducerArray velocity_message_to_transducer_array(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string& frame_id);

}  // namespace dvl_a50::ros

#endif  // ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP
