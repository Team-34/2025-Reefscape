#pragma once

#include <studica/AHRS.h>
#include <ctre/phoenix6/Pigeon2.hpp>

namespace t34 {

    class Gyro : public studica::AHRS{
    public: // METHODS
        static Gyro* Get();

        void ZeroYaw();

        void CreatePigeon();

    private: // METHODS
        Gyro();
    };
    
}