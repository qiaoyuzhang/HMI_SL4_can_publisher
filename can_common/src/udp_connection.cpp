#include "can_common/udp_connection.h"

#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <glog/logging.h>

namespace drive {
namespace common {
namespace can {

namespace {

const std::string kUDPSendAddress = "udp_send_address";
const std::string kUDPSendPort = "udp_send_port";
const std::string kUDPRecvAddress = "udp_recv_address";
const std::string kUDPRecvPort = "udp_recv_port";

bool InitializeUDPSocket(const ros::NodeHandle& config_source, const std::string& addr_prefix,
                         const std::string& port_prefix, const int can_index,
                         const std::string& default_address, const int default_port,
                         int& socket_fd, struct sockaddr_in& socket_addr) {
    // init socket fd
    if((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::stringstream ss;
        ss << __FILE__ << ":" << __LINE__ << "Opening UDP socket for CAN" << can_index << " failed";
        throw std::runtime_error(ss.str());
    }

    // read ip & port
    std::string udp_address = default_address;
    config_source.getParam(AppendIndex(addr_prefix, can_index), udp_address);
    in_addr_t s_addr = udp_address.empty() ? htonl(INADDR_ANY) : inet_addr(udp_address.data());
    int udp_port = default_port;
    config_source.getParam(AppendIndex(port_prefix, can_index), udp_port);

    // init sockaddr_in
    memset(&socket_addr, 0, sizeof(struct sockaddr_in));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = s_addr;
    socket_addr.sin_port = htons(udp_port);

    return true;
}

void VectorToCanFrame(const MessageID msg_id, const VectorDataType& data, unsigned char* frame) {
    frame[0] = 0x88;
    const MessageID n_msg_id = htonl(msg_id);
    memcpy(frame + 1, &n_msg_id, sizeof(n_msg_id));
    memcpy(frame + kUDPMessageHeaderLength, data.data(), kCANMessageLength);
}

void CANFrameToVector(const unsigned char* frame, MessageID& msg_id, VectorDataType& data) {
    msg_id = ntohl(*reinterpret_cast<const MessageID*>(frame + 1));
    memset(data.data(), 0, kCANMessageLength);
    memcpy(data.data(), frame + kUDPMessageHeaderLength, kCANMessageLength);
}

}

UDPConnection::UDPConnection(const int index)
    : ConnectionInterface(index), _recv_queue(128) {}

UDPConnection::~UDPConnection() {
    close(_send_socket_fd);
    close(_recv_socket_fd);
}

bool UDPConnection::Initialize(const ros::NodeHandle& config_source) {
    _type_name = can::ConnectionTypeToString(ConnectionType::UDP);

    if (InitializeUDPSocket(config_source, kUDPSendAddress, kUDPSendPort, _index,
                            "192.168.7.250", 4000, _send_socket_fd, _send_addr)) {
        LOG(INFO) << "UDP CAN" << _index << " send connection is initialized";
    } else {
        LOG(ERROR) << "UDP CAN" << _index << " send connection cannot be initialized";
        return false;
    }

    if (InitializeUDPSocket(config_source, kUDPRecvAddress, kUDPRecvPort, _index,
                            "", 8000, _recv_socket_fd, _recv_addr)) {
        struct timeval recv_timeout;
        recv_timeout.tv_sec = 0;  // timeout in 0.5 second
        recv_timeout.tv_usec = 500 * 1000;
        if (0 != setsockopt(_recv_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout,
                            sizeof(recv_timeout))) {
            int last_errno = errno;
            LOG(ERROR) << "Set timeout to UDP recv socket failed for CAN" << _index << ": "
                       << strerror(last_errno);
        }
        if (0 != bind(_recv_socket_fd, (struct sockaddr *)&_recv_addr, sizeof(_recv_addr))) {
            int last_errno = errno;
            LOG(ERROR) << "Bind to UDP recv socket failed for CAN" << _index << ": "
                       << strerror(last_errno);
            return false;
        }
        LOG(INFO) << "UDP CAN" << _index << " recv connection is initialized";
    } else {
        LOG(ERROR) << "UDP CAN" << _index << " recv connection cannot be initialized";
        return false;
    }

    return true;
}

bool UDPConnection::Start() {
    _running = true;
    _recv_thread = std::thread(&UDPConnection::ReadFromCanLoop, this);
    LOG(INFO) << "UDP CAN" << _index << " connection started";
    return true;
}

bool UDPConnection::Stop() {
    _running = false;
    _recv_thread.join();
    LOG(INFO) << "UDP CAN" << _index << " connection stopped";
    return true;
}

int UDPConnection::WriteToCan(const MessageID msg_id, const VectorDataType& data, const bool ) {
    unsigned char frame[kUDPMessageLength];
    VectorToCanFrame(msg_id, data, frame);
    const ssize_t result = sendto(_send_socket_fd, frame, kUDPMessageLength, 0,
                                 (struct sockaddr *)&_send_addr, sizeof(_send_addr));
    const int last_errno = errno;
    if (0 > result) {
        ConnectionTimeout(last_errno);
        LogSendFailure(msg_id, data);
    } else {
        ConnectionRecover();
    }
    return result;
}

int UDPConnection::ReadFromCan(MessageID& msg_id, VectorDataType& data) {
    std::unique_lock<std::mutex> lock(_recv_mx);
    _recv_cv.wait(lock, [&] { return !_running || !_recv_queue.empty(); });
    if (!_running) {
        return -1;
    }
    msg_id = _recv_queue.front().first;
    data = _recv_queue.front().second;
    _recv_queue.pop_front();
    return 1;
}

void UDPConnection::ReadFromCanLoop() {
    socklen_t recv_addr_len = sizeof(_recv_addr);
    constexpr int kBufferLength  = 1024;
    unsigned char recv_buff[kBufferLength];
    while(_running) {
        const int result = recvfrom(_recv_socket_fd, recv_buff, kBufferLength, 0,
                                   (struct sockaddr *)&_recv_addr, &recv_addr_len);
        const int last_errno = errno;
        if (0 > result) {
            ConnectionTimeout(last_errno);
        } else {
            MessageID msg_id;
            VectorDataType can_msg(kCANMessageLength);
            {
                std::lock_guard<std::mutex> lock_guard(_recv_mx);
                for(int i = 0; i < result; i += kUDPMessageLength) {
                    CANFrameToVector(recv_buff + i, msg_id, can_msg);
                    _recv_queue.push_back(std::make_pair(msg_id, can_msg));
                }
            }
            ConnectionRecover();
        }
        _recv_cv.notify_all();
    }
}

}  // namespace can
}  // namespace common
}  // namespace drive
