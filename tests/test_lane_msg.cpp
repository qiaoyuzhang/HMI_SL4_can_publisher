#include <gtest/gtest.h>
#include "hmi_message/lane_general_info_msg.h"

using namespace HMI::SL4::hmi_message;

TEST(LaneGeneralInfo, initialization) {
    
    LaneGeneralInfo lane_msg(false,true);
    
    ASSERT_EQ(lane_msg.getLeftLaneExist(), false);
    ASSERT_EQ(lane_msg.getRightLaneExist(), true);

    auto data_buffer = lane_msg.getVectorData();
    
    LaneGeneralInfo lane_msg2(data_buffer);

    ASSERT_EQ(lane_msg.getLeftLaneExist(), false);
    ASSERT_EQ(lane_msg.getRightLaneExist(), true);
 
}

TEST(LaneGeneralInfo, set_value) {
        
    LaneGeneralInfo lane_msg(false,true);
    
    lane_msg.setLeftLaneExist(true);
    lane_msg.setRightLaneExist(false);
    
    ASSERT_EQ(lane_msg.getLeftLaneExist(), true);
    ASSERT_EQ(lane_msg.getRightLaneExist(), false);

}
