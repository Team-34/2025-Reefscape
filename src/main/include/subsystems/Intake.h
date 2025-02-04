//uses sparkmax for now
#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

namespace t34
{
    class Intake 
    {
    public:
        virtual frc2::CommandPtr RunInCommand() = 0;
        virtual frc2::CommandPtr RunOutCommand() = 0;
    };
}