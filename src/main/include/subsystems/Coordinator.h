#pragma once

#include <frc2/command/SubsystemBase.h>
#include <subsystems/Elevator.h>
#include <subsystems/CoralIntake.h>
#include <commands/AutoDriveCommand.h>
#include "subsystems/SwerveDrive.h"
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <subsystems/AlgaeIntake.h>

namespace t34
{
    class Coordinator : public frc2::SubsystemBase 
    {
public:
    Coordinator(t34::Elevator* elevator, t34::CoralIntake* coral_intake, std::shared_ptr<t34::SwerveDrive> swerve_drive);


    void MoveToLevel(int level);

    frc2::CommandPtr NetPositionCommand();
    frc2::CommandPtr RunElevator(double speed);
    frc2::CommandPtr ScoreL3Auto();
    frc2::CommandPtr ScoreL4Auto();
    inline frc2::CommandPtr MoveUpLevelCommand() { return this->RunOnce([this]{ MoveToLevel(m_current_level + 1);}); }
    inline frc2::CommandPtr MoveDownLevelCommand() { return this->RunOnce([this]{ MoveToLevel(m_current_level - 1);}); }

    void Periodic() override;

private:
    t34::Elevator* m_elevator;
    t34::CoralIntake* m_coral_intake;
    std::shared_ptr<t34::SwerveDrive> m_swerve_drive;
    AutoDriveCommand m_basic_auto;
    int m_current_level;
    AlgaeIntake* m_algae_intake;
};
}
