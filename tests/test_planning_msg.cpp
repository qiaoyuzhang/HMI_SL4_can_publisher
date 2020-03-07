#include <gtest/gtest.h>
#include "hmi_message_publisher/planning_general_info_msg.h"

using namespace HMI::SL4::hmi_message;

TEST(PlanningGeneralInfo, initialization) {
    
    PlanningGeneralInfo planning_msg(false, false, true, 2, 55);
    ASSERT_EQ(planning_msg.getAllowLaneChange(), false);
    ASSERT_EQ(planning_msg.getLeftLaneChangeCmd(),false);
    ASSERT_EQ(planning_msg.getRightLaneChangeCmd(),true);
    ASSERT_EQ(planning_msg.getSetSpeed(),2);
    ASSERT_NEAR(planning_msg.getSpeedLimit(), 55 , PlanningGeneralInfo_speed_limit.resolution/2.0);

    auto data_buffer = planning_msg.getVectorData();
    PlanningGeneralInfo planning_msg2(data_buffer);
    
    ASSERT_EQ(planning_msg2.getAllowLaneChange(), false);
    ASSERT_EQ(planning_msg2.getLeftLaneChangeCmd(),false);
    ASSERT_EQ(planning_msg2.getRightLaneChangeCmd(),true);
    ASSERT_EQ(planning_msg2.getSetSpeed(),2);
    ASSERT_NEAR(planning_msg2.getSpeedLimit(), 55 , PlanningGeneralInfo_speed_limit.resolution/2.0);
   
}

TEST(PlanningGeneralInfo, set_value) {
    
    PlanningGeneralInfo planning_msg(false, false, false, 2, 55);
    
    planning_msg.setAllowLaneChange(true);
    planning_msg.setLeftLaneChangeCmd(true);
    planning_msg.setRightLaneChangeCmd(true);
    
    planning_msg.setSetSpeed(50);
    planning_msg.setSpeedLimit(61);
    
    
    
    ASSERT_EQ(planning_msg.getAllowLaneChange(), true);
    ASSERT_EQ(planning_msg.getLeftLaneChangeCmd(),true);
    ASSERT_EQ(planning_msg.getRightLaneChangeCmd(),true);
    ASSERT_NEAR(planning_msg.getSetSpeed(), 50 , PlanningGeneralInfo_set_speed.resolution/2.0);
    ASSERT_NEAR(planning_msg.getSpeedLimit(), 61 , PlanningGeneralInfo_speed_limit.resolution/2.0);
     
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
