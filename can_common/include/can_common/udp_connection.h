#pragma once

#include "can_common/connection.h"

#include <atomic>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <mutex>
#include <netinet/in.h>
#include <thread>

namespace drive {
namespace common {
namespace can {

/*
 * Connect with CAN throuth UDP, i.e. CAN-Ethernet gateway
 */
class UDPConnection final : public ConnectionInterface {
  public:
    explicit UDPConnection(const int index);
    ~UDPConnection();

    bool Initialize(const ros::NodeHandle& config_source) override;
    bool Start() override;
    bool Stop() override;
    int WriteToCan(const MessageID msg_id, const VectorDataType& data, const bool standard = false) override;
    // read one can message from the message queue, block if queue is empty and is running
    int ReadFromCan(MessageID& msg_id, VectorDataType& data) override;

  private:
    // one thread will be started and push new CAN messages to the message queue
    void ReadFromCanLoop();

    int _send_socket_fd = -1;
    struct sockaddr_in _send_addr;
    int _recv_socket_fd = -1;
    struct sockaddr_in _recv_addr;

    std::atomic_bool _running{ false };
    std::thread _recv_thread; // specific thread for receiving CAN messages
    using RecvData = std::pair<MessageID, VectorDataType>;
    boost::circular_buffer<RecvData> _recv_queue; // store the received CAN messages
    std::mutex _recv_mx;
    std::condition_variable _recv_cv;
};

}  // namespace can
}  // namespace common
}  // namespace drive
