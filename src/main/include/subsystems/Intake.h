//uses sparkmax
#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>

using namespace rev::spark;

namespace t34{
    
    class Intake : public frc2::SubsystemBase {
        public:
        
       Intake();

       frc2::CommandPtr Move(double input);
       
        private:

        SparkMax motor_1;
        SparkMax motor_2;

    };
}