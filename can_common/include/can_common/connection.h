#pragma once

#include <atomic>
#include <string>
#include <vector>

#include <ros/node_handle.h>

namespace drive {
namespace common {
namespace can {

/*
 * Common date types
 */
using MessageID = unsigned int;
using VectorDataType = std::vector<unsigned char>;
constexpr size_t kCANMessageLength = 8;
constexpr size_t kUDPMessageHeaderLength = 1 + sizeof(MessageID);
constexpr size_t kUDPMessageLength = kUDPMessageHeaderLength + kCANMessageLength;


/*
 * Interfaces for different kinds of CAN connections
 */
class ConnectionInterface {
  public:
    explicit ConnectionInterface(const int index) : _index(index) {}
    virtual ~ConnectionInterface() = default;

    virtual bool Initialize(const ros::NodeHandle& config_source) = 0;
    virtual bool Start() { return true; }
    virtual bool Stop() { return true; }
    virtual int WriteToCan(const MessageID msg_id, const VectorDataType& data, const bool standard = false) = 0;
    virtual int ReadFromCan(MessageID& msg_id, VectorDataType& data) = 0;
    virtual bool Timeout() const noexcept { return _timeout; }

    void SetIfName(const std::string& ifname) { _ifname = ifname; }
    std::string GetIfName() const noexcept { return _ifname; }

  protected:
    bool ConnectionFailEnoughTime(bool failure);
    void ConnectionTimeout(const int error_code);
    void ConnectionRecover();
    void LogSendFailure(const MessageID& msg_id, const VectorDataType& data);

    std::atomic_int  _fail_count{ 0 }; // how many connection failures encountered
    std::atomic_bool _timeout{ false };

    int _index = 0; // which CAN it connects to
    std::string _ifname; // the specified name for the connection
    std::string _type_name; // connection type name should be initialized by concrete connection
};


/*
 * Connection types
 */
enum class ConnectionType {
    UNKNOWN, // unknown connection type
    CAN, // connect directly with SocketCAN
    UDP, // connect with CAN-Ethernet gateway
};

std::string ConnectionTypeToString(const ConnectionType conn_type);
ConnectionType ConnectionTypeFromString(const std::string& str);


/*
 * Utilities
 */
// for example, given prefix="can_conn_type" and index=0, it will return "can_conn_type0"
std::string AppendIndex(const std::string& prefix, const int index);

// serialize input to vector char stream
template<typename InputType>
VectorDataType ToVectorData(const InputType& input) {
    VectorDataType output(sizeof(input));
    memcpy(output.data(), &input, sizeof(input));
    return output;
}

}  // namespace can
}  // namespace common
}  // namespace drive
