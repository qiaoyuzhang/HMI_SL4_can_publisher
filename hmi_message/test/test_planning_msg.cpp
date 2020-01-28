#include <gtest/gtest.h>
#include "hmi_message/planning_general_info_msg.h"

using namespace HMI::SL4::hmi_message;

TEST(PlanningGeneralInfo, initialization) {
    
    PlanningGeneralInfo planning_msg(false, 2, 55.5);
    ASSERT_EQ(planning_msg.getAllowLaneChange(), false);
    ASSERT_EQ(planning_msg.getTrajectoryType(), 2);
    ASSERT_NEAR(planning_msg.getSpeedLimit(), 55.5 , PlanningGeneralInfo_speed_limit.resolution/2.0); 

    auto data_buffer = planning_msg.getVectorData();
    PlanningGeneralInfo planning_msg2(data_buffer);
    
    ASSERT_EQ(planning_msg.getAllowLaneChange(), false);
    ASSERT_EQ(planning_msg.getTrajectoryType(), 2);
    ASSERT_NEAR(planning_msg.getSpeedLimit(), 55.5 , PlanningGeneralInfo_speed_limit.resolution/2.0); 
   
}

TEST(PlanningGeneralInfo, set_value) {
    
    PlanningGeneralInfo planning_msg(false, 2, 55.5);
    
    planning_msg.setAllowLaneChange(true);
    planning_msg.setTrajectoryType(3);
    planning_msg.setSpeedLimit(61.3);

    ASSERT_EQ(planning_msg.getAllowLaneChange(), true);
    ASSERT_EQ(planning_msg.getTrajectoryType(), 3);
    
    ASSERT_NEAR(planning_msg.getSpeedLimit(), 61.3 , PlanningGeneralInfo_speed_limit.resolution/2.0); 
    ASSERT_EQ(planning_msg.getSpeedLimit(), 61.0); 

    std::string name = TrajectoryType_descriptor->FindValueByNumber(planning_msg.getTrajectoryType())->name();
    ASSERT_EQ(name,"LANE_CHANGE_RIGHT");
}
