//uses sparkmax for now
#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
#include "Intake.h"

namespace t34
{
    class AlgaeIntake : public frc2::SubsystemBase, public Intake 
    {
    public:
        AlgaeIntake();

        virtual frc2::CommandPtr RunInCommand() override;
        virtual frc2::CommandPtr RunOutCommand() override;
        
    private:
        ctre::phoenix6::hardware::TalonFX m_motor;
    };
}