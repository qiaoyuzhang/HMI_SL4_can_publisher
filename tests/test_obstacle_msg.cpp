#include <gtest/gtest.h>
#include "hmi_message_publisher/obstacle_general_info_msg.h"
#include "hmi_message_publisher/obstacle_extended_info_msg.h"

using namespace HMI::SL4::hmi_message;

TEST(ObstacleExtendedInfo, initialization) {
    
    ObstacleExtendedInfo obstacle_msg(10, 0.5, 100, 2, false);
    ASSERT_EQ(obstacle_msg.getObId(), 10);
    ASSERT_EQ(obstacle_msg.getObClass(), 2);
    ASSERT_NEAR(obstacle_msg.getDistLong(), 0.5 , ObstacleExtendedInfo_dist_long.resolution/2.0); 
    ASSERT_NEAR(obstacle_msg.getDistLat(), 100, ObstacleExtendedInfo_dist_lat.resolution/2.0);
    ASSERT_EQ(obstacle_msg.getObThreat(), false);

    auto data_buffer = obstacle_msg.getVectorData();
    ObstacleExtendedInfo obstacle_msg2(data_buffer);
    
    ASSERT_EQ(obstacle_msg2.getObId(), 10);
    ASSERT_EQ(obstacle_msg2.getObClass(), 2);
    ASSERT_NEAR(obstacle_msg2.getDistLong(), 0.5 , ObstacleExtendedInfo_dist_long.resolution/2.0); 
    ASSERT_NEAR(obstacle_msg2.getDistLat(), 100, ObstacleExtendedInfo_dist_lat.resolution/2.0);
    ASSERT_EQ(obstacle_msg2.getObThreat(), false);

}

TEST(ObstacleExtendedInfo, set_value) {
    
    ObstacleExtendedInfo obstacle_msg(10, 0, 100, 2, false);
    
    
    obstacle_msg.setObId(1000);
    ASSERT_EQ(obstacle_msg.getObId(), 1000);

    obstacle_msg.setObClass(11);
    ASSERT_EQ(obstacle_msg.getObClass(), 11);
    std::string name = PerceptionObstacle_descriptor->FindValueByNumber(obstacle_msg.getObClass())->name();
    ASSERT_EQ(name,"CAR"); 
    
    obstacle_msg.setDistLong(ObstacleExtendedInfo_dist_long.max +  3 * ObstacleExtendedInfo_dist_long.resolution);
    ASSERT_EQ(obstacle_msg.getDistLong(), ObstacleExtendedInfo_dist_long.max);

    obstacle_msg.setDistLong(ObstacleExtendedInfo_dist_long.offset -  3 * ObstacleExtendedInfo_dist_long.resolution);
    ASSERT_EQ(obstacle_msg.getDistLong(), ObstacleExtendedInfo_dist_long.offset);

    obstacle_msg.setObThreat(true);
    ASSERT_EQ(obstacle_msg.getObThreat(), true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
