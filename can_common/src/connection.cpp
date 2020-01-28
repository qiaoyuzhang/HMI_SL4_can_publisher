#include "can_common/connection.h"

#include <strings.h>

#include <glog/logging.h>

namespace drive {
namespace common {
namespace can {

const char* kCAN = "CAN";
const char* kUDP = "UDP";
const char* kUnknown = "Unknown";

bool ConnectionInterface::ConnectionFailEnoughTime(const bool failure) {
    if (failure) {
        _fail_count++;
    } else {
        _fail_count = 0;
    }
    return _fail_count >= 5;
}

void ConnectionInterface::ConnectionTimeout(const int error_code) {
    if (ConnectionFailEnoughTime(true)) {
        if (!_timeout) {
            LOG(ERROR) << "CAN" << _index << " by " << _type_name << " timeout, error: "
                       << strerror(error_code);
        }
        _timeout = true;
    }
}

void ConnectionInterface::ConnectionRecover() {
    if (!ConnectionFailEnoughTime(false)) {
        if (_timeout) {
            LOG(INFO) << "CAN" << _index << " by " << _type_name << " recovered from timeout.";
        }
        _timeout = false;
    }
}

void ConnectionInterface::LogSendFailure(const MessageID& msg_id, const VectorDataType& data) {
    std::stringstream ss;
    ss << "Failed to send CAN frame with msg id: " << std::hex << std::uppercase << msg_id
       << ", data: [ ";
    for (auto v : data) {
        ss << static_cast<unsigned int>(v) << " ";
    }
    ss << "] for CAN" << _index;
    LOG(ERROR) << ss.str().c_str();
}

std::string ConnectionTypeToString(const ConnectionType conn_type) {
    switch(conn_type) {
    case ConnectionType::CAN:
        return kCAN;
    case ConnectionType::UDP:
        return kUDP;
    default:
        return kUnknown;
    }
}

ConnectionType ConnectionTypeFromString(const std::string& str) {
    if (::strcasecmp(str.data(), kCAN) == 0) {
        return ConnectionType::CAN;
    } else if (::strcasecmp(str.data(), kUDP) == 0) {
        return ConnectionType::UDP;
    } else {
        return ConnectionType::UNKNOWN;
    }
}

std::string AppendIndex(const std::string& prefix, const int index) {
    return prefix + std::to_string(index);
}

}  // namespace can
}  // namespace common
}  // namespace drive
