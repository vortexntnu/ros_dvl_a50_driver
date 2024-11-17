#ifndef ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP
#define ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP

#include "lib/types.hpp"

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace dvl_a50::ros
{

geometry_msgs::msg::TwistWithCovarianceStamped translate_velocity_message(const dvl_a50::lib::VelocityMessage& msg);
geometry_msgs::msg::PoseWithCovarianceStamped translate_position_local_message(const dvl_a50::lib::PositionLocalMessage& msg);

} // namespace dvl_a50::ros

#endif // ROS_DVL_A50_DRIVER_ROS_ROS_TRANSLATOR_HPP
