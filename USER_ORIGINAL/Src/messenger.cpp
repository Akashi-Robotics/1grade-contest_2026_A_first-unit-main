#include "messenger.hpp"
#include "Config.hpp"
#include "main.h"
#include "CAN.hpp"
#include "CAN.hpp"
#include "stm32g4xx_hal_fdcan.h"

Messenger::Messenger() : packet{0, 0, 1, 1, 1} {
}                                                       //CANを開始 シリンダを開放 ベルコンを待機位置に移動

void cylinder(CYLINDER_OPERATION_MODE mode, CYLINDER_STATE state)
{
    // シリンダの操作
    if(CYLINDER_STATE == REVERSE) packet[mode] = !packet[mode];
    else packet[mode] = state;
    send();
}

void conveyor (CONVEYOR_OPERATION_MODE mode, CONVEYOR_STATE state)
{
    // コンベアの操作
    if(CONVEYOR_STATE == PLUS && packet[mode + 2] < 3) packet[mode + 2] ++;
    else if(CONVEYOR_STATE == MINUS && packet[mode + 2] > 0) packet[mode + 2] --;
    else packet[mode + 2] = state;
    send();
}


//-------------------------------define Sender----------------------------//
void Messenger::send(){
    CAN_ns::send(&hfdcan2, 0x001, packet);
}
