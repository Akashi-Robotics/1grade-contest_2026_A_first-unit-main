#include "driving.hpp"
#include "Config.hpp"
#include "main.h"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "pid.hpp"
#include "BNO.hpp"
#include "i2c.h"
#include "math.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"

Driving::Driving() 
: Vx(0.0), Vy(0.0), yaw(0.0) ,
  motor{Motor(&htim1, TIM_CHANNEL_1, &htim1, TIM_CHANNEL_2, POSITIVE, POSITIVE), //Motor0
        Motor(&htim1, TIM_CHANNEL_3, &htim1, TIM_CHANNEL_4, POSITIVE, POSITIVE), //Motor1
        Motor(&htim2, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_2, POSITIVE, POSITIVE), //Motor2
        Motor(&htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4, POSITIVE, POSITIVE)}, //Motor3
  bno(&hi2c3),
  encoder{Encoder_SINGLE_Interrupt(GPIOB, GPIO_PIN_2, GPIOB, GPIO_PIN_1, 512, POSITIVE),  //Encoder0
          Encoder_SINGLE_Interrupt(GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, 512, POSITIVE), //Encoder1
          Encoder_SINGLE_Interrupt(GPIOB, GPIO_PIN_13, GPIOC, GPIO_PIN_5, 512, POSITIVE), //Encoder2
          Encoder_SINGLE_Interrupt(GPIOB, GPIO_PIN_9, GPIOA, GPIO_PIN_7, 512, POSITIVE)}, //Encoder3
    pid{Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f),
        Vel_PID(0.0015f, 0.008f, 0.0002f)}
{    
    for(int i = 0; i < 4; i++)
    {
        resolution[i] = static_cast<float>(motor[i].get_resolution());
        pid[i].set_limits(-0.5f, 0.5f);
    }
}

void Driving::set_velocity(double Vx_, double Vy_, bool R1_) {
    Vx = Vx_;
    Vy = Vy_;
    R1 = R1_;
    bno.get_yaw_angle();
    yaw = bno.yaw_angle;

    // printf("Vx: %4d, Vy: %4d, yaw: %2d  ", static_cast<int>(Vx), static_cast<int>(Vy), static_cast<int>(yaw * 100));

    for(int i = 0; i < 4; i++){
            encoder[i].update();
            value[i] = static_cast<double>(encoder[i].get_omega()); // Update encoder values
            // printf("%5d", static_cast<int>(value[i] * 100.0));

            target[i] = (-(Vx / 128.0f) * sin(theta[i] + yaw) + (Vy / 128.0f) * cos(theta[i] + yaw) - R * yaw) / r; // Calculate target values
            if(R1 == true){
                target[i] *= 0.3;
            }
            pid[i].Input(target[i], value[i]); // Update PID controller
            motor[i].set_value(static_cast<int32_t>(pid[i].Output() * resolution[i])); // Set motor values
    }
    // printf("\n");
    
}