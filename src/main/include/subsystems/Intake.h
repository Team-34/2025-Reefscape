//uses sparkmax for now
#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

namespace t34{
    
    class Intake : public frc2::SubsystemBase {
        public:
        
        Intake();

        frc2::CommandPtr RunIn(double power_percentage);
        frc2::CommandPtr RunOut(double power_percentage);
        
        private:

        SparkMax motor_1;
        SparkMax motor_2;
    };
}