#include "ros_node.hpp"
#include "ros_translator.hpp"

namespace dvl_a50::ros
{

DvlA50DriverNode::DvlA50DriverNode()
    : Node("dvl_a50_driver_node")
{
    this->declare_parameter<std::string>("ip_address", "192.168.194.95");
    this->declare_parameter<int>("port", 16171);
    this->declare_parameter<std::string>("frame_id", "dvl_link");

    std::string ip_address = this->get_parameter("ip_address").as_string();
    int port = this->get_parameter("port").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose", 10);

    auto velocity_callback = [this](const dvl_a50::lib::VelocityMessage& msg) {
        publish_twist(msg);
    };

    auto position_callback = [this](const dvl_a50::lib::PositionLocalMessage& msg) {
        publish_pose(msg);
    };

    dvl_driver_ = std::make_shared<dvl_a50::lib::DvlA50Driver>(ip_address, port,
                                                               velocity_callback,
                                                               position_callback);
}

void DvlA50DriverNode::publish_twist(const dvl_a50::lib::VelocityMessage& msg)
{
    auto twist_msg = velocity_message_to_twist(msg, frame_id_);
    twist_publisher_->publish(twist_msg);
}

void DvlA50DriverNode::publish_pose(const dvl_a50::lib::PositionLocalMessage& msg)
{
    auto pose_msg = position_local_to_pose(msg, frame_id_);
    pose_publisher_->publish(pose_msg);
}

} // namespace dvl_a50::ros
