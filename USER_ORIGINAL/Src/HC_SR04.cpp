#include "HC_SR04.hpp"
#include "Config.hpp"
#include "main.h"
#include "Interrupts.hpp"
#include "stm32g4xx_hal_fdcan.h"
#include "CAN.hpp"

HC_SR04::HC_SR04() : distanceData{0, 0} {
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan1);
} // コンストラクタでdistanceDataを初期化

void HC_SR04::set_distance(uint8_t data0, uint8_t data1)//cpp_main.cppの中でCAN通信で受信したデータをこの関数に渡すことで、距離データを更新する
{
    distanceData.RxData[0] = data0;
    distanceData.RxData[1] = data1;
}

float HC_SR04::get_distance()
{
    return static_cast<float>(distanceData.distance) / 100.0f; //受信時に1/100(mm)に圧縮されたデータを還元
}