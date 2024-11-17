#include "lib/tcp_client.hpp"
#include "lib/parser.hpp"


using boost::asio::ip::tcp;

namespace dvl_a50::lib
{

TcpClient::TcpClient(const std::string &ip, int port,
                     VelocityMessageCallback velocity_callback,
                     PositionLocalMessageCallback position_callback)
    : io_context_(), socket_(io_context_), stopped_(false),
      velocity_callback_(velocity_callback), position_callback_(position_callback) {

    tcp::endpoint endpoint(boost::asio::ip::make_address(ip), port);
    boost::system::error_code ec;
    socket_.connect(endpoint, ec);
    if (ec) {
        return;
    }

    buffer_.clear();
    start_reading();

    io_thread_ = std::thread([this]() {
        io_context_.run();
    });
}

TcpClient::~TcpClient() {
    stop();
}

void TcpClient::stop() {
    if (stopped_) return;
    stopped_ = true;
    boost::system::error_code ec;
    socket_.shutdown(tcp::socket::shutdown_both, ec);
    socket_.close(ec);
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void TcpClient::start_reading() {
    auto buf = std::make_shared<std::vector<char>>(1024);
    socket_.async_read_some(boost::asio::buffer(*buf),
        [this, buf](const boost::system::error_code &ec, std::size_t bytes_transferred) {
            if (!ec) {
                std::string data(buf->data(), bytes_transferred);
                buffer_ += data;
                process_buffer();
                if (!stopped_) {
                    start_reading();
                }
            } else if (ec != boost::asio::error::operation_aborted) {
            }
        });
}

void TcpClient::process_buffer() {
    dvl_a50::lib::process_buffer(buffer_, velocity_callback_, position_callback_);
}

} // namespace dvl_a50::lib
