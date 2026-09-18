#include "Config.hpp"
#include "Interrupts.hpp"
#include "Timer.hpp"
#include "main.h"
#include <cstdio>
#include "CAN.hpp"
#include "fdcan.h"
#include "ACS_Wireless_920MHz.hpp"
#include "stm32g4xx_hal_fdcan.h"
#include "BNO.hpp"
#include "i2c.h"
#include "driving.hpp"
// #include "HC_SR04.hpp"

// HC_SR04 hc_sr04;

int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.05f;

    Timer timer;
    float elapsed_time = 0.0f;
    timer.reset();
    timer.start();

    int Vx = 0;
    int Vy = 0;
    bool R1 = 0;

    main_timer::activate();

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan2);
    ACS_Wireless_920MHz::ACS_Device ctler(&hfdcan2, 3);

    Driving driving;

    while (true)
    {
        printf("%d   ", ctler.is_alive());

        // control omni
        Vx = ctler.get_Lx() - 128;
        Vy = - (ctler.get_Ly() - 128);
        R1 = ctler.get_R1();
        if((Vx < 10) && (Vx > -10)) Vx = 0;
        if((Vy < 10) && (Vy > -10)) Vy = 0;
        driving.set_velocity(Vx, Vy, R1);

        elapsed_time = timer.read();
        while(timer.read() - elapsed_time < cycle::dt);
    }

    return 0;
}

// void USER_HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan) // 受信の際はこの関数が呼び出されるのでここにコードを書いてください。重い処理を書かないように。
// {
//     if(hfdcan == &hfdcan1 && RxMessage.RxHeader.Identifier == 0x123) // can1から0x123のメッセージが来た時に以下を実行
//     {
//         hc_sr04.set_distance(RxMessage.RxData[0], RxMessage.RxData[1]); // 受信したデータをHC_SR04クラスのset_distance関数に渡す
//     }
    
// }