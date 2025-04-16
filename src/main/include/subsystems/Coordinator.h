#pragma once

#include <frc2/command/SubsystemBase.h>
#include <subsystems/Elevator.h>
#include <subsystems/CoralIntake.h>
#include <commands/AutoDriveCommand.h>
#include "subsystems/SwerveDrive.h"
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>

namespace t34
{
    class Coordinator : public frc2::SubsystemBase 
    {
public:
    Coordinator(t34::Elevator* elevator, t34::CoralIntake* coral_intake);


    void MoveToLevel(int level);

    frc2::CommandPtr RunElevator(double speed);
    //frc2::CommandPtr ScoreL3Auto();
    inline frc2::CommandPtr MoveUpLevelCommand() {return this->RunOnce([this]{ MoveToLevel(m_current_level + 1);}); }
    inline frc2::CommandPtr MoveDownLevelCommand() {return this->RunOnce([this]{ MoveToLevel(m_current_level - 1);}); }

    void Periodic() override;

private:
    t34::Elevator* m_elevator;
    t34::CoralIntake* m_coral_intake;
    int m_current_level;
    AutoDriveCommand m_basic_auto;
    std::shared_ptr<t34::SwerveDrive> swerve_drive;

};
}
