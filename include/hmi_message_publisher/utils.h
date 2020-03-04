#pragma once
#include <glog/logging.h>
#include <vector>
#include <unordered_map>
#include <list>

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

/*
 * Least Recently Used Cache
 * get(key): get the value from the key, return -1 if map didn't have this key
 * put(key): put the key in the map and automatically assigned a least used value( in the range of 0 to _cacheCapacity) to it.
*/
class LRUCache {
  public:
    LRUCache(const unsigned int capacity):
        _cacheCapacity(capacity)
        {}

    int get(int key) {
        // return -1, if map didn't find the item
        if (_map.find(key) == _map.end()){
            return -1;
        }
        // move it to the front
        _list.push_front(std::make_pair(key, _map[key]->second));
        _list.erase(_map[key]);
        _map[key] = _list.begin();
        return _map[key]->second;
    }

    int put(int key) {
        // check if it already exist in cache list. If yes, move it to front
        if (_map.find(key) != _map.end()) {
            return get(key);
        } else {
            // check capacity
            if (_map.size() < _cacheCapacity) {
                // space is still available
                // use map.size as the value
                unsigned int value = _map.size();
                // insert it to cache list and make its entry in the map
                _list.push_front(std::make_pair(key, value));
                _map[key] = _list.begin();
                return value;
            } else {
                // map is at capacity limit, so remove the last (LRU) entry in cache list, and reuse it's value
                unsigned int value = _list.back().second;
                _map.erase(_list.back().first);
                _list.pop_back();
                // add the new entry to cache list and make its entry in the map
                _list.push_front(std::make_pair(key, value));
                _map[key] = _list.begin();
                return value;
            }
        }
    }
  private:
    std::unordered_map<int, std::list<std::pair<int, unsigned int>>::iterator> _map;
    std::list<std::pair<int,unsigned int>> _list;
    unsigned int _cacheCapacity;
};


}
}
}
