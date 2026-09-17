#ifndef DRIVING_HPP
#define DRIVING_HPP

#include "pid.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "BNO.hpp"
#include "math.h"

// Your code here
class Driving{
    
    public:
        Driving();
        void set_velocity(double Vx_, double Vy_);
    private:
        double Vx;
        double Vy;
        double yaw;
        double value[4];
        double target[4] = {0.0, 0.0, 0.0, 0.0};
        Motor motor[4];
        BNO055 bno;
        Encoder_SINGLE_Interrupt encoder[4];
        Vel_PID pid[4];
        int resolution[4];
        static constexpr double theta[4] = {(33.7/180.0) * M_PI, (146.3/180.0) * M_PI, (213.7/180.0) * M_PI, (326.3/180.0) * M_PI};
        static constexpr double R = 0.54;
        static constexpr double r = 0.053;
};


#endif // DRIVING_HPP