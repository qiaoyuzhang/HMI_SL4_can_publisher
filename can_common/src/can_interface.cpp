#include "can_common/can_interface.h"

// TODO: QNX, CAN interface on QNX is different from Linux, need a seperate PR for QNX CAN
#if !defined(__QNX__) && !defined(__QNXNTO__)
#include "can_common/can_connection.h"
#endif
#include "can_common/udp_connection.h"

#include <glog/logging.h>

namespace drive {
namespace common {

namespace {

const std::string kCanConnType = "can_conn_type";
const std::string kIfName = "ifname";

}

CanInterface::CanInterface(const int index) {
    if (Init(index)) {
        LOG(INFO) << "CanInterface " << index << " Initialization Succeeded.";
    } else {
        LOG(ERROR) << "CanInterface " << index << " Initialization Failed.";
        std::exit(EXIT_FAILURE);
    }
}

bool CanInterface::Init(const int index) {
    const ros::NodeHandle config_source("~");
    // can connection type
    std::string can_conn_type_str;
    can::ConnectionType can_conn_type = can::ConnectionType::UNKNOWN;
    if (config_source.getParam(can::AppendIndex(kCanConnType, index), can_conn_type_str)) {
        can_conn_type = can::ConnectionTypeFromString(can_conn_type_str);
    }
    switch(can_conn_type) {
        case can::ConnectionType::UDP:
            _connection.reset(new can::UDPConnection(index));
            break;
        case can::ConnectionType::CAN:
        default: // default to can connection for backward compatibility
// TODO: QNX, CAN interface on QNX is different from Linux, need a seperate PR for QNX CAN
#if !defined(__QNX__) && !defined(__QNXNTO__)
            _connection.reset(new can::CANConnection(index));
#else
            LOG(ERROR) << "Direct connect with CAN on QNX has not been developed!";
            std::exit(EXIT_FAILURE);
#endif
            break;
    }

    // ifname
    std::string ifname;
    if (config_source.getParam(can::AppendIndex(kIfName, index), ifname)) {
        _connection->SetIfName(ifname);
    }

    // initialize specific fields
    return _connection->Initialize(config_source);
}

}  // namespace common
}  // namespace drive
