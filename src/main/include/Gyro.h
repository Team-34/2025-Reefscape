#pragma once

#include <studica/AHRS.h>
#include <ctre/phoenix6/Pigeon2.hpp>
namespace t34 {

    class Gyro : public ctre::phoenix6::hardware::Pigeon2 {
    public: // METHODS
        static Gyro* Get();

        void ZeroYaw();

    private: // METHODS
        Gyro();
    };
    
}