#ifndef ROS_DVL_A50_DRIVER_LIB_TCP_CLIENT_HPP
#define ROS_DVL_A50_DRIVER_LIB_TCP_CLIENT_HPP

#include <boost/asio.hpp>
#include <thread>
#include <string>
#include <functional>

#include "lib/parser.hpp"

namespace dvl_a50::lib
{

class TcpClient {
public:
    TcpClient(const std::string &ip, int port,
              VelocityMessageCallback velocity_callback,
              PositionLocalMessageCallback position_callback);
    ~TcpClient();
    void stop();

private:
    void start_reading();
    void process_buffer();

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket socket_;
    std::string buffer_;
    std::thread io_thread_;
    bool stopped_;

    VelocityMessageCallback velocity_callback_;
    PositionLocalMessageCallback position_callback_;
};

} // namespace dvl_a50::lib

#endif  // ROS_DVL_A50_DRIVER_LIB_TCP_CLIENT_HPP
