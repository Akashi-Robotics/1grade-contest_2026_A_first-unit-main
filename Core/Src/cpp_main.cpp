#include "Config.hpp"
#include "Interrupts.hpp"
#include "Timer.hpp"
#include "main.h"
#include <cstdio>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "pid.hpp"
#include "CAN.hpp"
#include "fdcan.h"
#include "main.h"
#include <cstdio>
#include "ACS_Wireless_920MHz.hpp"
#include "stm32g4xx_hal_fdcan.h"
#include "BNO.hpp"
#include "i2c.h"
#include "math.h"

constexpr double R = 0.54;
constexpr double r = 0.053;


char ctler_data[8] = {0};

// void USER_HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan)
// {
//     printf("%d\n", (int)RxMessage.RxHeader.Identifier);
//     // if(RxMessage.RxHeader.Identifier == 154)
//     // {
//     //     for(int i = 0; i < 8; i++)
//     //     {
//     //         ctler_data[i] = RxMessage.RxData[i];
//     //         printf("%02d   ", (int)ctler_data[i]);
//     //     }
//     //     printf("\n");
//     // }
// }

int cpp_main()
{
    main_timer::activate();
    cycle::dt = 0.05f;

    Timer timer;
    float elapsed_time = 0.0f;
    timer.reset();
    timer.start();

    // BNO055 bno(&hi2c3);
    float yaw = 0.0f;
    
    Motor motor[4] = {
        Motor (&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2, POSITIVE, POSITIVE), //Motor0
        Motor (&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4, POSITIVE, POSITIVE), //Motor1
        Motor (&htim2, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_2, POSITIVE, POSITIVE), //Motor2
        Motor (&htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4, POSITIVE, POSITIVE)  //Motor3
    };

    Encoder_SINGLE_Interrupt encoder[4] = {
        Encoder_SINGLE_Interrupt (GPIOB, GPIO_PIN_9, GPIOA, GPIO_PIN_7, 512, POSITIVE),  //Encoder0
        Encoder_SINGLE_Interrupt (GPIOB, GPIO_PIN_13, GPIOC, GPIO_PIN_5, 512, POSITIVE), //Encoder1
        Encoder_SINGLE_Interrupt (GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, 256, POSITIVE), //Encoder2
        Encoder_SINGLE_Interrupt (GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_1, 512, POSITIVE) //Encoder3
    };

    Vel_PID pid[4] = {
        Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f)
    };

    float resolution[4];
    for(int i = 0; i < 4; i++)
    {
        resolution[i] = static_cast<float>(motor[i].get_resolution());
        pid[i].set_limits(-0.5f, 0.5f);
    }

    double value[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Vx = 0;
    float Vy = 0;
    int Omega = 0;


    main_timer::activate();

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    CAN_ns::set_all_pass_filter(&hfdcan2);
    ACS_Wireless_920MHz::ACS_Device ctler(&hfdcan2, 3);

    while (true)
    {
        printf("%d   ", ctler.is_alive());
        //get information
        for(int i = 0; i < 4; i++)
        {
            encoder[i].update();
            value[i] = static_cast<double>(encoder[i].get_omega());
            printf("%6d", static_cast<int>(value[i] * 100));
        }

        Vx = ctler.get_Lx() - 128;
        Vy = - (ctler.get_Ly() - 128);
        if((Vx < 10) && (Vx > -10)) Vx = 0;
        if((Vy < 10) && (Vy > -10)) Vy = 0;
        Omega = (ctler.get_L2() - ctler.get_R2());

        // printf("%4d   ", static_cast<int>(yaw * 100));
        // printf("%4d   ", static_cast<int>(Vx));
        // printf("%4d   ", static_cast<int>(Vy));
        // printf("%4d\n", static_cast<int>(Omega));

        // bno.get_yaw_angle();
        // yaw = bno.yaw_angle;;

        // printf("%d   ", ctler.is_alive());
        // printf("%d   ", static_cast<int>(100 * Vx));
        // printf(" %d\n", static_cast<int>(100 * Vy));

        //PID
        for(int i = 0; i < 2; i++)
        {
            target[2 * i] = (-(Vx / 128.0f) * sin(M_PI * (33.7f / 180.0f) + M_PI * i + yaw) + (Vy / 128.0f) * cos(M_PI * (33.7f / 180.0f) + M_PI * i + yaw) + R * (Omega / 255.0f)) / r;
            pid[2 * i].Input(target[2 * i], value[2 * i]);
            motor[2 * i].set_value(static_cast<int32_t>(pid[2 * i].Output() * resolution[2 * i]));

            target[2 * i + 1] = (-(Vx / 128.0f) * sin(M_PI * (146.3f / 180.0f) + M_PI * i + yaw) + (Vy / 128.0f) * cos(M_PI * (146.3f / 180.0f) + M_PI * i + yaw) + R * (Omega / 255.0f)) / r;
            pid[2 * i + 1].Input(target[2 * i + 1], value[2 * i + 1]);
            motor[2 * i + 1].set_value(static_cast<int32_t>(pid[2 * i + 1].Output() * resolution[2 * i + 1]));
        }

        elapsed_time = timer.read();
        while(timer.read() - elapsed_time < cycle::dt);
    }

    return 0;
}