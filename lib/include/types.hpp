#ifndef ROS_DVL_A50_DRIVER_LIB_TYPES_HPP
#define ROS_DVL_A50_DRIVER_LIB_TYPES_HPP

#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include <nlohmann/json.hpp>

namespace dvl_a50::lib
{

struct Transducer
{
    int id;
    double velocity;
    double distance;
    double rssi;
    double nsd;
    bool beam_valid;
};

struct VelocityMessage
{
    double time;
    double vx;
    double vy;
    double vz;
    double fom;
    std::array<std::array<double, 3>, 3> covariance;
    double altitude;
    std::vector<Transducer> transducers;
    bool velocity_valid;
    int status;
    std::string format;
    std::string type;
    int64_t time_of_validity;
    int64_t time_of_transmission;
};

struct PositionLocalMessage
{
    double ts;
    double x;
    double y;
    double z;
    double std;
    double roll;
    double pitch;
    double yaw;
    std::string type;
    int status;
    std::string format;
};

} // namespace dvl_a50::lib

namespace nlohmann
{

inline void from_json(const nlohmann::json& j, dvl_a50::lib::Transducer& t)
{
    j.at("id").get_to(t.id);
    j.at("velocity").get_to(t.velocity);
    j.at("distance").get_to(t.distance);
    j.at("rssi").get_to(t.rssi);
    j.at("nsd").get_to(t.nsd);
    j.at("beam_valid").get_to(t.beam_valid);
}

inline void from_json(const nlohmann::json& j, dvl_a50::lib::VelocityMessage& msg)
{
    j.at("time").get_to(msg.time);
    j.at("vx").get_to(msg.vx);
    j.at("vy").get_to(msg.vy);
    j.at("vz").get_to(msg.vz);
    j.at("fom").get_to(msg.fom);

    const auto& cov_json = j.at("covariance");
    for (size_t i = 0; i < msg.covariance.size(); ++i) {
        cov_json.at(i).get_to(msg.covariance[i]);
    }

    j.at("altitude").get_to(msg.altitude);
    j.at("transducers").get_to(msg.transducers);
    j.at("velocity_valid").get_to(msg.velocity_valid);
    j.at("status").get_to(msg.status);
    j.at("format").get_to(msg.format);
    j.at("type").get_to(msg.type);
    j.at("time_of_validity").get_to(msg.time_of_validity);
    j.at("time_of_transmission").get_to(msg.time_of_transmission);
}

inline void from_json(const nlohmann::json& j, dvl_a50::lib::PositionLocalMessage& msg)
{
    j.at("ts").get_to(msg.ts);
    j.at("x").get_to(msg.x);
    j.at("y").get_to(msg.y);
    j.at("z").get_to(msg.z);
    j.at("std").get_to(msg.std);
    j.at("roll").get_to(msg.roll);
    j.at("pitch").get_to(msg.pitch);
    j.at("yaw").get_to(msg.yaw);
    j.at("type").get_to(msg.type);
    j.at("status").get_to(msg.status);
    j.at("format").get_to(msg.format);
}

} // namespace nlohmann

#endif  // ROS_DVL_A50_DRIVER_LIB_TYPES_HPP
