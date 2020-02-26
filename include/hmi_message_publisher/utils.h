#include <glog/logging.h>
#include <vector>

namespace HMI{
namespace SL4{
namespace hmi_message{
//  write first n bits of data to data buffer's [ start_pos : start_pos + n) bit
static void writeDataToDataVector(std::vector<unsigned char>& data_buffer, const unsigned char& start_pos, const unsigned char& n, const unsigned int& data){
        size_t end_pos = start_pos + n;
        if (data_buffer.size() * 8 < end_pos){
            LOG(ERROR) << "Trying to write data from " << (int)start_pos << " to " << (int)(start_pos + n) << " : larger then data_buffer size: " << data_buffer.size() * 8;
            return;
        }
         
        for(unsigned char i = 0 ; i < n ; i++) {
            size_t row = (end_pos - i -1) / 8;
            size_t pos = (end_pos - i -1) % 8;
            
            unsigned char data_buffer_mask = ~ ( 1 << ( 7 - pos));
            unsigned int data_mask = 1 << i;
            unsigned char filter_data = (data & data_mask) >> i;
            data_buffer[row] = (data_buffer[row] & data_buffer_mask) | (filter_data << ( 7 - pos));
        }
}
// get Data from specified position and length 
static void getDataFromDataVector(const std::vector<unsigned char>& data_buffer, const unsigned char& start_pos, const unsigned char& n, unsigned int& data){
        size_t end_pos = start_pos + n;
        if (data_buffer.size() * 8 < end_pos){
            LOG(ERROR) << "Trying to get data from " << (int)start_pos << " to " << (int)(start_pos + n) << " : larger then data_buffer size: " << data_buffer.size() * 8;
            return;
        }
        data = 0; 
        for(unsigned char i = 0 ; i < n ; i++) {
            size_t row = (end_pos - i -1 ) / 8;
            size_t pos = (end_pos - i -1 ) % 8;
            unsigned char data_buffer_mask = ( 1 << ( 7 - pos));
            data +=  (((data_buffer[row] & data_buffer_mask) >> (7-pos))<< i);
        }
}

}
}
}
