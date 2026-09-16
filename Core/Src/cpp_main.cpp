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

constexpr double R = 0.54;
constexpr double r = 0.053;



namespace sensor{
    union distanceData{ 
      uint8_t RxData[2];
     uint16_t distance;
    };
    distanceData distanceData;
}

int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.05f;

    Timer timer;
    float elapsed_time = 0.0f;
    timer.reset();
    timer.start();

    float Vx = 0;
    float Vy = 0;

    main_timer::activate();

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan2);
    ACS_Wireless_920MHz::ACS_Device ctler(&hfdcan2, 3);

    Driving driving;

    while (true)
    {
        printf("%d   ", ctler.is_alive());

        Vx = ctler.get_Lx() - 128;
        Vy = - (ctler.get_Ly() - 128);
        if((Vx < 10) && (Vx > -10)) Vx = 0;
        if((Vy < 10) && (Vy > -10)) Vy = 0;

        // printf("%4d   ", static_cast<int>(yaw * 100));
        // printf("%4d   ", static_cast<int>(Vx));
        // printf("%4d   ", static_cast<int>(Vy));
        // printf("%4d\n", static_cast<int>(Omega));

        driving.set_velocity(Vx, Vy);

        elapsed_time = timer.read();
        while(timer.read() - elapsed_time < cycle::dt);
    }

    return 0;
}

void USER_HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan) // 受信の際はこの関数が呼び出されるのでここにコードを書いてください。重い処理を書かないように。
{
    if(hfdcan == &hfdcan1 && RxMessage.RxHeader.Identifier == 0x123) // can1から0x123のメッセージが来た時に以下を実行
    {
        sensor::distanceData.RxData[0] = RxMessage.RxData[0];
        sensor::distanceData.RxData[1] = RxMessage.RxData[1];
        // printf("%d\n", distanceData.distance);
    }
    
}