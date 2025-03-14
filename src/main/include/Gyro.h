#pragma once

#include <studica/AHRS.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
namespace t34 {

    class Gyro : public ctre::phoenix6::hardware::Pigeon2 {
    public: // METHODS
        static Gyro* Get();

        void ZeroYaw();

        //frc2::CommandPtr ZeroYawCommand();
        
    private: // METHODS
        Gyro();
    };
    
}