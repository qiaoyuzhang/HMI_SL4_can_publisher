#include "hmi_message_publisher/can_node.h"
#include <glog/logging.h>
using namespace HMI::SL4::hmi_message_publisher;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "hmi_message_publisher_can_node");
    CanNode can_node;
    can_node.Run();
}
