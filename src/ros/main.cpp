#include "ros/ros_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<dvl_a50::ros::DvlA50DriverNode>();

    std::cout << R"(
         ______     ___            _    ____   ___  
        |  _ \ \   / / |          / \  | ___| / _ \ 
        | | | \ \ / /| |   _____ / _ \ |___ \| | | |
        | |_| |\ V / | |__|_____/ ___ \ ___) | |_| |
        |____/  \_/ _|_____|   /_/   \_\____/ \___/ 
        |  _ \ _ __(_)_   _____ _ __                
        | | | | '__| \ \ / / _ \ '__|               
        | |_| | |  | |\ V /  __/ |                  
        |____/|_|  |_| \_/ \___|_|                  
    )" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
