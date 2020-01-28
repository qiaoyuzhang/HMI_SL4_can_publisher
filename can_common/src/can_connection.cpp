#include "can_common/can_connection.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <glog/logging.h>

namespace drive {
namespace common {
namespace can {

namespace {

const std::string kExtendedID = "extended_id";
const std::string kSocketBindErrorFilter = "socket_bind_error_filter";

void VectorToCanFrame(const MessageID msg_id, const VectorDataType& data, can_frame &frame) {
    frame.can_id = msg_id;
    frame.can_dlc = std::min(kCANMessageLength, data.size());  // Number of bytes of data (0–8 bytes)
    memset(frame.data, 0, kCANMessageLength);
    memcpy(frame.data, data.data(), frame.can_dlc);
}

void CANFrameToVector(const can_frame &frame, MessageID& msg_id, VectorDataType& data) {
    msg_id = frame.can_id;
    data.resize(kCANMessageLength, 0);
    memcpy(data.data(), frame.data, frame.can_dlc);
}

}

CANConnection::~CANConnection() {
    close(_socket_fd);
}

bool CANConnection::Initialize(const ros::NodeHandle& config_source) {
    _type_name = can::ConnectionTypeToString(ConnectionType::CAN);

    config_source.getParam(kExtendedID, _extended_id);

    if ((_socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::stringstream ss;
        ss << __FILE__ << ":" << __LINE__ << " Opening CAN socket failed.";
        throw std::runtime_error(ss.str());
    }
    ifreq ifr_;
    strcpy(ifr_.ifr_name, _ifname.c_str());
    ioctl(_socket_fd, SIOCGIFINDEX, &ifr_);
    sockaddr_can addr_;
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(_socket_fd, (sockaddr *)&addr_, sizeof(addr_)) < 0) {
        int last_errno = errno;
        std::stringstream ss;
        ss << __FILE__ << ":" << __LINE__ << ":" << " CAN socket bind fail, error: "
           << strerror(last_errno);
        bool socket_bind_error_filter = false;
        config_source.getParam(kSocketBindErrorFilter, socket_bind_error_filter);
        LOG(ERROR) << "CAN connection " << ifr_.ifr_name << " at index: " << ifr_.ifr_ifindex
                   << " bind error, " << strerror(errno);
        if (socket_bind_error_filter == false) {
            throw std::runtime_error(ss.str());
        }
    }
    LOG(INFO) << "CAN connection " << ifr_.ifr_name << " at index: " << ifr_.ifr_ifindex
              << " is initialized";
    return true;
}

int CANConnection::WriteToCan(const MessageID msg_id, const VectorDataType& data, const bool standard) {
    can_frame frame;
    VectorToCanFrame(msg_id, data, frame);
    if (_extended_id == true && standard == false) {
        frame.can_id |= CAN_EFF_FLAG;  // 0x80000000
    } else {
        frame.can_id &= CAN_SFF_MASK;
    }
    const ssize_t result = write(_socket_fd, &frame, CAN_MTU);
    const int last_errno = errno;
    if (0 > result) {
        ConnectionTimeout(last_errno);
        LogSendFailure(msg_id, data);
    } else {
        ConnectionRecover();
    }
    return result;
}

int CANConnection::ReadFromCan(MessageID& msg_id, VectorDataType& data) {
    can_frame frame;
    const ssize_t result = read(_socket_fd, &frame, CAN_MTU);
    const int last_errno = errno;
    if (0 > result) {
        ConnectionTimeout(last_errno);
    } else {
        CANFrameToVector(frame, msg_id, data);
        msg_id &= 0x7fffffff;  // force bit 32 to '0'
        ConnectionRecover();
    }
    return result;
}

}  // namespace can
}  // namespace common
}  // namespace drive
