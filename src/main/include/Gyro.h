#pragma once

#include <studica/AHRS.h>

namespace t34 {

    class Gyro : public studica::AHRS {
    public: // METHODS
        static Gyro* Get();

        void ZeroYaw();

    private: // METHODS
        Gyro();
    };
    
}