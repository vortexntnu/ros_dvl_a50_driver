#include "driver.hpp"

#include <boost/asio/error.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

using boost::asio::ip::tcp;

namespace dvl_a50::lib {

void process_message(const std::string &data,
                     const VelocityMessageCallback &velocity_callback,
                     const PositionLocalMessageCallback &position_callback) {

  nlohmann::json msg;
  try {
    msg = nlohmann::json::parse(data);
  } catch (nlohmann::json::parse_error &e) {
    spdlog::error("Error parsing JSON: {}", e.what());
    return;
  }

  if (!msg.contains("type") || !msg["type"].is_string()) {
    spdlog::warn("Message received without valid 'type' field, skipping...");
    return;
  }

  const auto &type = msg["type"].get<std::string>();

  if (type == "velocity") {
    velocity_callback(msg.get<VelocityMessage>());
  } else if (type == "position_local") {
    position_callback(msg.get<PositionLocalMessage>());
  } else {
    spdlog::warn("Unknown message type '{}' received, skipping...", type);
  }
}

DvlA50Driver::DvlA50Driver(const std::string &ip, int port,
                           VelocityMessageCallback velocity_callback,
                           PositionLocalMessageCallback position_callback)
    : io_context_(), socket_(io_context_),
      velocity_callback_(velocity_callback),
      position_callback_(position_callback) {

  tcp::endpoint endpoint(boost::asio::ip::make_address(ip), port);
  boost::system::error_code ec;
  socket_.connect(endpoint, ec);
  if (ec) {
    spdlog::error("Failed to connect: {}", ec.message());
    return;
  }

  read();

  io_thread_ = std::thread([this]() { io_context_.run(); });
}

DvlA50Driver::~DvlA50Driver() {
  boost::system::error_code ec;
  socket_.shutdown(tcp::socket::shutdown_both, ec);
  socket_.close(ec);
  io_context_.stop();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void DvlA50Driver::read() {
  auto buf = std::make_shared<std::vector<char>>(buffer_size_);
  socket_.async_read_some(boost::asio::buffer(*buf),
                          [this, buf](const boost::system::error_code &ec,
                                      std::size_t bytes_transferred) {
                            if (!ec) {
                              std::string data(buf->data(), bytes_transferred);
                              process_message(data, velocity_callback_,
                                              position_callback_);
                              read();
                            } else {
                              spdlog::error("Read error: {}", ec.message());
                            }
                          });
}

} // namespace dvl_a50::lib
