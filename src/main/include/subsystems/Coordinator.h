#pragma once

#include <iostream>
#include "subsystems/Climber.h"
#include "subsystems/Elevator.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/AlgaeIntake.h"
#include <frc2/command/SubsystemBase.h>
#include <RobotContainer.h>
#include <frc2/command/CommandPtr.h>

using namespace frc2;
namespace t34 {
  class Coordinator : public frc2::SubsystemBase {
    public:
    Coordinator();

    CommandPtr TestPathCommand();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    private:
    Elevator m_elevator;
    CoralIntake m_coral_intake;
    AlgaeIntake m_algae_intake;
  };
}