#include <gtest/gtest.h>
#include "hmi_message/utils.h"

using namespace HMI::SL4::hmi_message;

TEST(Utils, get_data) {
    std::vector<unsigned char> data_vector(8);
    data_vector[0] = 0xFF; 
    data_vector[1] = 0xF1;
    data_vector[2] = 0x97;       
    data_vector[3] = 0x35;       
    data_vector[4] = 0x73;       
    data_vector[5] = 0xDD;       
    data_vector[6] = 0xAB;       
    data_vector[7] = 0x30;       
    
    unsigned int data0 = 0;
    getDataFromDataVector(data_vector, 0, 8, data0);   
    ASSERT_EQ(data_vector[0], data0);    
    
    unsigned int data1 = 0;
    getDataFromDataVector(data_vector, 8, 8, data1);   
    ASSERT_EQ(data_vector[1], data1);    
    
    unsigned int data3 = 0;
    getDataFromDataVector(data_vector, 24, 8, data3);   
    ASSERT_EQ(data_vector[3], data3);    
    
    unsigned int data5 = 0;
    getDataFromDataVector(data_vector, 40, 8, data5);   
    ASSERT_EQ(data_vector[5], data5);    
    
    unsigned int data7 = 0;
    getDataFromDataVector(data_vector, 56, 8, data7);   
    ASSERT_EQ(data_vector[7], data7);
    
    unsigned int data = 0;
    getDataFromDataVector(data_vector, 0, 1, data);
    ASSERT_EQ(1, data);
    
    getDataFromDataVector(data_vector, 7, 4, data);
    ASSERT_EQ(15, data);
    
    getDataFromDataVector(data_vector, 11, 5, data);
    ASSERT_EQ(17, data);
    
    getDataFromDataVector(data_vector, 0, 16, data);
    ASSERT_EQ(65521, data);
       
}

TEST(Utils, convert_data) {
    std::vector<unsigned char> data_vector(8);
    data_vector[0] = 0xFF; 
    data_vector[1] = 0xF1;
    data_vector[2] = 0x97;       
    data_vector[3] = 0x35;       
    data_vector[4] = 0x73;       
    data_vector[5] = 0xDD;       
    data_vector[6] = 0xAB;       
    data_vector[7] = 0x30;       
    
    unsigned int data = 253;
    writeDataToDataVector(data_vector, 7, 8, data);   
    unsigned int rec_data = 0;
    getDataFromDataVector(data_vector, 7, 8, rec_data);
    ASSERT_EQ(rec_data, data);

    data = 6;
    writeDataToDataVector(data_vector, 22, 3, data);   
    rec_data = 0;
    getDataFromDataVector(data_vector, 22, 3, rec_data);
    ASSERT_EQ(rec_data, data);
    
    data = 31663;
    writeDataToDataVector(data_vector, 37, 15, data);   
    rec_data = 0;
    getDataFromDataVector(data_vector, 37, 15, rec_data);
    ASSERT_EQ(rec_data, data);   
    
}
