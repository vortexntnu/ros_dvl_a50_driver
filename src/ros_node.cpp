#include <ros_dvl_a50_driver/ros_node.hpp>
#include <ros_dvl_a50_driver/ros_translator.hpp>

namespace dvl_a50::ros {

DvlA50DriverNode::DvlA50DriverNode(const rclcpp::NodeOptions& options)
    : Node("dvl_a50_driver_node", options) {
    const auto launch_message = R"(
         ______     ___            _    ____   ___
        |  _ \ \   / / |          / \  | ___| / _ \
        | | | \ \ / /| |   _____ / _ \ |___ \| | | |
        | |_| |\ V / | |__|_____/ ___ \ ___) | |_| |
        |____/  \_/ _|_____|   /_/   \_\____/ \___/
        |  _ \ _ __(_)_   _____ _ __
        | | | | '__| \ \ / / _ \ '__|
        | |_| | |  | |\ V /  __/ |
        |____/|_|  |_| \_/ \___|_|
    )";
    spdlog::info(launch_message);

    this->declare_parameter<std::string>("ip_address", "192.168.194.95");
    this->declare_parameter<int>("port", 16171);
    this->declare_parameter<std::string>("frame_id", "dvl_link");
    this->declare_parameter<std::string>("twist_topic", "dvl/twist");
    this->declare_parameter<std::string>("pose_topic", "dvl/pose");
    this->declare_parameter<std::string>("altitude_topic", "dvl/altitude");

    std::string ip_address = this->get_parameter("ip_address").as_string();
    int port = this->get_parameter("port").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    std::string twist_topic = this->get_parameter("twist_topic").as_string();
    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    std::string altitude_topic =
        this->get_parameter("altitude_topic").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            twist_topic, qos_sensor_data);
    pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos_sensor_data);
    altitude_publisher_ = this->create_publisher<vortex_msgs::msg::DVLAltitude>(
        altitude_topic, qos_sensor_data);

    auto velocity_callback = [this](const dvl_a50::lib::VelocityMessage& msg) {
        publish_twist(msg);
        publish_altitude(msg);
    };

    auto position_callback =
        [this](const dvl_a50::lib::PositionLocalMessage& msg) {
            publish_pose(msg);
        };

    dvl_driver_ = std::make_shared<dvl_a50::lib::DvlA50Driver>(
        ip_address, port, velocity_callback, position_callback);
}

void DvlA50DriverNode::publish_twist(const dvl_a50::lib::VelocityMessage& msg) {
    auto twist_msg = velocity_message_to_twist(msg, frame_id_);
    twist_publisher_->publish(twist_msg);
}

void DvlA50DriverNode::publish_pose(
    const dvl_a50::lib::PositionLocalMessage& msg) {
    auto pose_msg = position_local_to_pose(msg, frame_id_);
    pose_publisher_->publish(pose_msg);
}

void DvlA50DriverNode::publish_altitude(
    const dvl_a50::lib::VelocityMessage& msg) {
    auto altitude_msg = velocity_message_to_altitude(msg, frame_id_);
    altitude_publisher_->publish(altitude_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(dvl_a50::ros::DvlA50DriverNode)

}  // namespace dvl_a50::ros
