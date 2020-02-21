#pragma once
#include <math.h>
namespace HMI {
namespace SL4 {
namespace hmi_message{

//  message ids
static const unsigned int VehicleStatusGeneralInfoMsgId = 0x300;
static const unsigned int ObstacleGeneralInfoMsgId = 0x400;
static const unsigned int LaneGeneralInfoMsgId = 0x500;
static const unsigned int PlanningGeneralInfoMsgId = 0x600;

/* HMI SL4 message data configuration
 * start: the start position of the value
 * len: length of the value
 * define resolution and minimum of message
 * actual value =  pow(2, value) * resolution + offset
 * maximum data = pow(2,len)
 */

struct DataConfigInt{
    const unsigned char data_start = 0;
    const unsigned char len = 0;
    const int offset = 0;
    const unsigned int resolution = 1;
    const int max = 0;
    
    DataConfigInt(const unsigned char& data_start, const unsigned char& len, const int& offset, const unsigned int& resolution):
        data_start(data_start),
        len(len),
        offset(offset),
        resolution(resolution),
        max((pow(2.0, len)-1) * resolution + offset)
    {}
    int recoverValue(const unsigned int& data) const {
        return data*resolution + offset;
    }
    unsigned int getData(const int& value) const {
        unsigned int data = 0;
        if (value <= offset){
             data = 0;
        }
        else if ( value >= max){
             data = (1 << len) - 1;
        }
        else {
            data = ( value - offset) / resolution;
        }
        return data;
    }
};

struct DataConfigDouble{
    const unsigned char data_start = 0;
    const unsigned char len = 0;
    const double offset = 0;
    const double resolution = 1;
    const double max = 0;
    
    DataConfigDouble(const unsigned char& data_start, const unsigned char& len, const double& offset, const double& resolution):
        data_start(data_start),
        len(len),
        offset(offset),
        resolution(resolution),
        max((pow(2.0, len)-1) * resolution + offset)
        {}
    double recoverValue(const unsigned int& data) const {
        return data*resolution + offset;
    }
    unsigned int getData (const int& value) const {
        unsigned int data = 0;
        if (value <= offset){
             data = 0;
        }
        else if ( value >= max){
             data = (1 << len) - 1;
        }
        else {
            data = (value - offset) / resolution;
        }
        return data;
    }
};    

struct DataConfigBool{
    const unsigned char data_start = 0;
    const unsigned char len = 1;
    DataConfigBool(const unsigned char& data_start):
        data_start(data_start){}
    bool recoverValue(const unsigned int& data) const {
        return data == 0? false : true;
    }
    unsigned int getData(const bool& value) const {
        return value? 1: 0;
    }
};


//  get the minimum number of byte to storage the n bits
static unsigned int n_byte(const unsigned int n_bit){
    return ((n_bit%8) == 0 )? n_bit/8 :  (n_bit/8) + 1;
};


// VehicleStatus
static const DataConfigInt VehicleStatusGeneralInfo_speed(0,10,-256,1);
static const DataConfigInt VehicleStatusGeneralInfo_steering_angle(10,11,-1024,1);
static const DataConfigInt VehicleStatusGeneralInfo_engage_status(21,2,0,1);

static const unsigned int VehicleStatusGeneralInfo_data_total_bit = 23; //sum of all lens in the message
static const unsigned int VehicleStatusGeneralInfo_data_size = n_byte(VehicleStatusGeneralInfo_data_total_bit);

//Obstacle

static const DataConfigInt ObstacleGeneralInfo_ob_id(0,12,0,1);
static const DataConfigDouble ObstacleGeneralInfo_dist_long(12,12,-2048,1);
static const DataConfigDouble ObstacleGeneralInfo_dist_lat(24,12,-200,0.5);
static const DataConfigInt ObstacleGeneralInfo_ob_class(36,5,0,1);
static const DataConfigBool ObstacleGeneralInfo_ob_threat(41);

static const unsigned int ObstacleGeneralInfo_data_total_bit = 42; //sum of all lens in the message
static const unsigned int ObstacleGeneralInfo_data_size = n_byte(ObstacleGeneralInfo_data_total_bit);

//Lane

static const DataConfigBool LaneGeneralInfo_left_lane_exist(0);
static const DataConfigBool LaneGeneralInfo_right_lane_exist(1);

static const unsigned int LaneGeneralInfo_data_total_bit = 2; //sum of all lens in the message
static const unsigned int LaneGeneralInfo_data_size = n_byte(LaneGeneralInfo_data_total_bit);


//Planning

static const DataConfigBool PlanningGeneralInfo_allow_lane_change(0);
static const DataConfigBool PlanningGeneralInfo_left_lane_change_cmd(1);
static const DataConfigBool PlanningGeneralInfo_right_lane_change_cmd(2);
static const DataConfigDouble PlanningGeneralInfo_set_speed(3,8,0,0.3);
static const DataConfigDouble PlanningGeneralInfo_speed_limit(11,8,0,0.3);

static const unsigned int PlanningGeneralInfo_data_total_bit = 19; //sum of all lens in the message
static const unsigned int PlanningGeneralInfo_data_size = n_byte(PlanningGeneralInfo_data_total_bit);

}
}  // namespace SL4
}  // namespace HMI
