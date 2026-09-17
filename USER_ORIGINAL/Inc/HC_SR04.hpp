#ifndef HC_SR04_HPP
#define HC_SR04_HPP

#include "main.h"

//write your code here
union sensor{ 
    uint8_t RxData[2];
    uint16_t distance;
};

class HC_SR04{
    public:
        HC_SR04();
        float get_distance();
        void set_distance(uint8_t data0, uint8_t data1);
    private:
        sensor distanceData;
};

#endif // HC_SR04_HPP