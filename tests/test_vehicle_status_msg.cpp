#include <gtest/gtest.h>
#include "hmi_message_publisher/vehicle_status_general_info_msg.h"

using namespace HMI::SL4::hmi_message;

TEST(VehicleStatusGeneralInfo, initialization) {
    VehicleEngageStatus engage_status = NOT_READY;
    double speed_test =
            VehicleStatusGeneralInfo_speed.offset - VehicleStatusGeneralInfo_speed.resolution;
    VehicleStatusGeneralInfo vehicle_status1(speed_test, 100, engage_status);
    ASSERT_EQ(VehicleStatusGeneralInfo_speed.offset, vehicle_status1.getSpeed());
    ASSERT_EQ(100, vehicle_status1.getSteeringAngle());
    ASSERT_EQ(engage_status, vehicle_status1.getEngageStatus());

    engage_status = READY;
    speed_test = VehicleStatusGeneralInfo_speed.max + VehicleStatusGeneralInfo_speed.resolution;
    VehicleStatusGeneralInfo vehicle_status2(speed_test, 300, engage_status);
    ASSERT_EQ(VehicleStatusGeneralInfo_speed.max, vehicle_status2.getSpeed());
    ASSERT_EQ(300, vehicle_status2.getSteeringAngle());
    ASSERT_EQ(engage_status, vehicle_status2.getEngageStatus());

    engage_status = ENGAGED;
    speed_test = 100;
    VehicleStatusGeneralInfo vehicle_status3(speed_test, 0, engage_status);
    ASSERT_EQ(speed_test, vehicle_status3.getSpeed());
    ASSERT_EQ(0, vehicle_status3.getSteeringAngle());
    ASSERT_EQ(engage_status, vehicle_status3.getEngageStatus());

    VehicleStatusGeneralInfo rec_vehicle_status3(vehicle_status3.getVectorData());
    ASSERT_EQ(rec_vehicle_status3.getSpeed(), vehicle_status3.getSpeed());
    ASSERT_EQ(rec_vehicle_status3.getSteeringAngle(), vehicle_status3.getSteeringAngle());
    ASSERT_EQ(rec_vehicle_status3.getEngageStatus(), vehicle_status3.getEngageStatus());
}

TEST(VehicleStatusGeneralInfo, set_value) {
    VehicleEngageStatus engage_status = NOT_READY;
    VehicleStatusGeneralInfo vehicle_status(100, 0, engage_status);

    vehicle_status.setSpeed(20);

    ASSERT_EQ(20, vehicle_status.getSpeed());

    vehicle_status.setSteeringAngle(200);

    ASSERT_EQ(200, vehicle_status.getSteeringAngle());

    vehicle_status.setEngageStatus(VehicleEngageStatus::ENGAGED);
    ASSERT_EQ(VehicleEngageStatus::ENGAGED, vehicle_status.getEngageStatus());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
