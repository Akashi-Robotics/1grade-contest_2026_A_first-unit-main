#include "messenger.hpp"
#include "Config.hpp"
#include "main.h"
#include "CAN.hpp"
#include "CAN.hpp"
#include "stm32g4xx_hal_fdcan.h"

Messenger::Messenger() : packet{0,1,1} {
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan1);
}                                                       //CANを開始 シリンダを開放 ベルコンを待機位置に移動

//-------------------------------define Air------------------------------//
void Messenger::cylinder(CYLINDER_STATE state)
{
    packet[0] = state;
    send();
}

void Messenger::small_conveyor(CONVEYOR_STATE state)
{
    packet[1] = state;
    send();
}

void Messenger::big_conveyor(CONVEYOR_STATE state)
{
    packet[2] = state;
    send();
}

//-------------------------------define Sender----------------------------//
void Messenger::send(){
    CAN_ns::send(&hfdcan1, 0x001, packet);
}
