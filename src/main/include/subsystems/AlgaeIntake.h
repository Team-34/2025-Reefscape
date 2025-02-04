//uses sparkmax for now
#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
#include "Intake.h"
using namespace rev::spark;

namespace t34
{
    class AlgaeIntake : public frc2::SubsystemBase, public Intake 
    {
    public:
        AlgaeIntake();

        virtual frc2::CommandPtr RunInCommand() override;
        virtual frc2::CommandPtr RunOutCommand() override;
        
    private:
        SparkMax m_primary_motor;
        SparkMax m_secondary_motor;
    };
}