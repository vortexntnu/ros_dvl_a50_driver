#ifndef ROS_DVL_A50_DRIVER_LIB_PARSER_HPP
#define ROS_DVL_A50_DRIVER_LIB_PARSER_HPP

#include <functional>
#include <nlohmann/json.hpp>
#include <string>

#include "lib/types.hpp"

namespace dvl_a50::lib {

using VelocityMessageCallback = std::function<void(const VelocityMessage &)>;
using PositionLocalMessageCallback =
    std::function<void(const PositionLocalMessage &)>;

void process_buffer(std::string &buffer,
                    VelocityMessageCallback velocity_callback,
                    PositionLocalMessageCallback position_callback);

} // namespace dvl_a50::lib

#endif // ROS_DVL_A50_DRIVER_LIB_PARSER_HPP
