#include "Gyro.h"

#include "Constants.h"
#include <ctre/phoenix6/Pigeon2.hpp>

namespace t34 {

    static std::unique_ptr<Gyro> g_gyro{ nullptr };

    Gyro* Gyro::Get() {
        if (!g_gyro) {
            g_gyro.reset(new Gyro());
        }

        return g_gyro.get();
    }

    void Gyro::CreatePigeon() {
        ctre::phoenix6::hardware::Pigeon2 Pigeon_Gyro(1,"RIO");
    }

    /**
     * Resets the Gyro Z (Yaw) axis to a heading of zero. 
     * This can be used if there is significant drift in 
     * the gyro and it needs to be recalibrated after it 
     * has been running.
     */
    void Gyro::ZeroYaw() { 
        Reset(); 
    }

}