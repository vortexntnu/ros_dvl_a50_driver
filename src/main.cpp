#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include "ros_node.hpp"

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::debug);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<dvl_a50::ros::DvlA50DriverNode>();

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

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
