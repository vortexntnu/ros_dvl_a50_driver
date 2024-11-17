#include "ros/ros_node.hpp"

namespace dvl_a50::ros
{

DvlA50DriverNode::DvlA50DriverNode()
    : Node("dvl_a50_driver_node")
{
    this->declare_parameter<std::string>("ip_address", "192.168.194.95");
    this->declare_parameter<int>("port", 16171);

    std::string ip_address = this->get_parameter("ip_address").as_string();
    int port = this->get_parameter("port").as_int();

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose", 10);

    auto velocity_callback = [this](const dvl_a50::lib::VelocityMessage& msg) {
        publish_twist(msg);
    };

    auto position_callback = [this](const dvl_a50::lib::PositionLocalMessage& msg) {
        publish_pose(msg);
    };

    tcp_client_ = std::make_shared<dvl_a50::lib::TcpClient>(ip_address, port,
                                                               velocity_callback,
                                                               position_callback);
}

DvlA50DriverNode::~DvlA50DriverNode()
{
    if (tcp_client_) {
        tcp_client_->stop();
    }
}

void DvlA50DriverNode::publish_twist(const dvl_a50::lib::VelocityMessage& msg)
{
    auto twist_msg = translate_velocity_message(msg);
    twist_publisher_->publish(twist_msg);
}

void DvlA50DriverNode::publish_pose(const dvl_a50::lib::PositionLocalMessage& msg)
{
    auto pose_msg = translate_position_local_message(msg);
    pose_publisher_->publish(pose_msg);
}

} // namespace dvl_a50::ros
