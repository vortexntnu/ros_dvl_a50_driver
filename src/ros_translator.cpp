#include <ros_dvl_a50_driver/ros_translator.hpp>

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

namespace dvl_a50::ros {

double to_rad(const double deg) {
    return deg * M_PI / 180.0;
}

geometry_msgs::msg::TwistWithCovarianceStamped velocity_message_to_twist(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string& frame_id) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;

    twist_msg.header.stamp = rclcpp::Clock().now();
    twist_msg.header.frame_id = frame_id;

    twist_msg.twist.twist.linear.x = msg.vx;
    twist_msg.twist.twist.linear.y = msg.vy;
    twist_msg.twist.twist.linear.z = msg.vz;

    twist_msg.twist.covariance.fill(0.0);

    const auto dim = msg.covariance.size();
    for (std::size_t i = 0; i < dim; ++i) {
        for (std::size_t j = 0; j < dim; ++j) {
            // Need to stride by 2 * dim since the twist covariance is 6x6
            twist_msg.twist.covariance[i * 2 * dim + j] = msg.covariance[i][j];
        }
    }

    return twist_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped position_local_to_pose(
    const dvl_a50::lib::PositionLocalMessage& msg,
    const std::string frame_id) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = rclcpp::Clock().now();
    pose_msg.header.frame_id = frame_id;

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

    // std (fom) isn't perfect for this, but it's the best we have /shrug
    pose_msg.pose.covariance[0] = msg.std * msg.std;
    pose_msg.pose.covariance[7] = msg.std * msg.std;
    pose_msg.pose.covariance[14] = msg.std * msg.std;

    return pose_msg;
}

vortex_msgs::msg::DVLAltitude velocity_message_to_altitude(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string frame_id) {
    vortex_msgs::msg::DVLAltitude altitude_msg;
    altitude_msg.header.stamp = rclcpp::Clock().now();
    altitude_msg.header.frame_id = frame_id;
    altitude_msg.altitude = msg.altitude;

    return altitude_msg;
}

vortex_msgs::msg::TransducerArray velocity_message_to_transducer_array(
    const dvl_a50::lib::VelocityMessage& msg,
    const std::string& frame_id) {

    vortex_msgs::msg::TransducerArray transducer_array_msg;

    transducer_array_msg.header.stamp = rclcpp::Clock().now();
    transducer_array_msg.header.frame_id = frame_id;

    transducer_array_msg.transducers.resize(msg.transducers.size());

    auto convert_transducer = [](const dvl_a50::lib::Transducer &in) -> vortex_msgs::msg::Transducer {
        vortex_msgs::msg::Transducer out;
        out.id         = in.id;
        out.velocity   = in.velocity;
        out.distance   = in.distance;
        out.rssi       = in.rssi;
        out.nsd        = in.nsd;
        out.beam_valid = in.beam_valid;
        return out;
    };

    std::transform(msg.transducers.begin(), msg.transducers.end(),
                   transducer_array_msg.transducers.begin(),
                   convert_transducer);

    transducer_array_msg.time_of_validity = msg.time_of_validity;
    transducer_array_msg.time_of_transmission = msg.time_of_transmission;

    return transducer_array_msg;
}

}  // namespace dvl_a50::ros
