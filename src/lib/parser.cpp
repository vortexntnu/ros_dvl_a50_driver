#include "lib/parser.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace dvl_a50::lib
{

void process_message(const json& msg,
                     const VelocityMessageCallback& velocity_callback,
                     const PositionLocalMessageCallback& position_callback) {

    auto type_it = msg.find("type");
    if (type_it == msg.end() || !type_it->is_string()) {
        return;
    }

    const std::string& type = *type_it;

    if (type == "velocity") {
        const auto velocity_msg = msg.get<dvl_a50::lib::VelocityMessage>();
        velocity_callback(velocity_msg);

    } else if (type == "position_local") {
        const auto position_msg = msg.get<dvl_a50::lib::PositionLocalMessage>();
        position_callback(position_msg);
    } else {
        // TODO: warn log unknown message
    }
}

// TODO: Rewrite buffer processing code

size_t find_matching_brace(const std::string& str, size_t start) {
    int brace_count = 0;
    for (size_t i = start; i < str.size(); ++i) {
        if (str[i] == '{') {
            brace_count++;
        } else if (str[i] == '}') {
            brace_count--;
            if (brace_count == 0) {
                return i;
            }
        }
    }
    return std::string::npos; // No matching brace found
}

void process_buffer(std::string& buffer,
                    VelocityMessageCallback velocity_callback,
                    PositionLocalMessageCallback position_callback) {

    while (true) {
        size_t start = buffer.find('{');
        if (start == std::string::npos) {
            buffer.clear();
            break;
        }
        size_t end = find_matching_brace(buffer, start);
        if (end == std::string::npos) {
            break;
        }
        std::string json_str = buffer.substr(start, end - start + 1);

        buffer = buffer.substr(end + 1);

        try {
            auto j = json::parse(json_str);
            process_message(j, velocity_callback, position_callback);
        } catch (json::parse_error& e) {
            // TODO: warn log parse error
        }
    }
}

} // namespace dvl_a50::lib
