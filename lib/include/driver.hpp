#ifndef ROS_DVL_A50_DRIVER_LIB_DRIVER_HPP
#define ROS_DVL_A50_DRIVER_LIB_DRIVER_HPP

#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <functional>

#include "types.hpp"

namespace dvl_a50::lib
{

using VelocityMessageCallback = std::function<void(const VelocityMessage&)>;
using PositionLocalMessageCallback = std::function<void(const PositionLocalMessage&)>;

class DvlA50Driver {
public:
    DvlA50Driver(const std::string &ip, int port,
              VelocityMessageCallback velocity_callback,
              PositionLocalMessageCallback position_callback);
    ~DvlA50Driver();

private:
    void read();

    static constexpr size_t buffer_size_ = 4096;

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket socket_;
    std::string buffer_;
    std::thread io_thread_;

    VelocityMessageCallback velocity_callback_;
    PositionLocalMessageCallback position_callback_;
};
} // namespace dvl_a50::lib

#endif  // ROS_DVL_A50_DRIVER_LIB_DRIVER_HPP
