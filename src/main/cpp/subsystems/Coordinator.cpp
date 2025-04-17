#include "subsystems/Coordinator.h"

t34::Coordinator::Coordinator(t34::Elevator* elevator, t34::CoralIntake* coral_intake, std::shared_ptr<t34::SwerveDrive> swerve_drive)
: m_elevator(elevator)
, m_coral_intake(coral_intake)
, m_swerve_drive(swerve_drive)
, m_basic_auto(m_swerve_drive, 0_in, 3_ft, 0_deg)
, m_current_level(0)

{
    
}

void t34::Coordinator::MoveToLevel(int level)
{
    frc::SmartDashboard::PutNumber("Level moving to", level);

    m_current_level = std::clamp(level, 0, 3);

    switch (m_current_level)
    {
    case 0:
        m_elevator->ElevateTo(0.0);
        break;
    case 1:
        m_elevator->ElevateTo(1.586);
        break;
    case 2:
        m_elevator->ElevateTo(5.3);
        break;
    case 3:
        m_elevator->ElevateTo(9.99);
        m_coral_intake->MoveWristTo(9.357);
        break;
    }

}

frc2::CommandPtr t34::Coordinator::ScoreL3Auto()
{
    return AutoDriveCommand{m_swerve_drive, 0_in, 3_ft, 0_deg}.ToPtr() //Leave and move up to the reef
        .AndThen(frc2::cmd::Wait(1.5_s))
        .AndThen(this->RunOnce( [this] { MoveToLevel(2); } )) //Bring elevator up to score an L3
        .AndThen(m_coral_intake->MoveWristToCommand(12.0)) //Move coral wrist into position
        .AndThen(frc2::cmd::Wait(1.25_s))
        .AndThen(m_coral_intake->RunOutCommand(0.5) //Spit out coral
            .WithDeadline(frc2::cmd::Wait(2_s))) //Keep running for 1 second
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(m_coral_intake->StopIntakeCommand())
        .AndThen(m_coral_intake->MoveToZero()) //Return coral intake back to the top
        .AndThen(AutoDriveCommand{m_swerve_drive, 0_in, -1_ft, 0_deg}.ToPtr()) //Move backward
        .AndThen(frc2::cmd::Wait(0.3_s))
        .AndThen(this->RunOnce( [this] { MoveToLevel(0); } )); //Return elevator back to starting position
}

frc2::CommandPtr t34::Coordinator::ScoreL2Auto()
{
    return AutoDriveCommand{m_swerve_drive, 0_in, 3_ft, 0_deg}.ToPtr() //Leave and move up to the reef
        .AndThen(frc2::cmd::Wait(1.5_s))
        .AndThen(this->RunOnce( [this] { MoveToLevel(1); } )) //Bring elevator up to score an L3
        .AndThen(m_coral_intake->MoveWristToCommand(12.0)) //Move coral wrist into position
        .AndThen(frc2::cmd::Wait(1.25_s))
        .AndThen(m_coral_intake->RunOutCommand(0.5) //Spit out coral
            .WithDeadline(frc2::cmd::Wait(2_s))) //Keep running for 1 second
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(m_coral_intake->StopIntakeCommand())
        .AndThen(m_coral_intake->MoveToZero()) //Return coral intake back to the top
        .AndThen(AutoDriveCommand{m_swerve_drive, 0_in, -1_ft, 0_deg}.ToPtr()) //Move backward
        .AndThen(frc2::cmd::Wait(0.3_s))
        .AndThen(this->RunOnce( [this] { MoveToLevel(0); } )); //Return elevator back to starting position
}


// frc2::CommandPtr t34::Coordinator::NetPositionCommand()
// {
//     return this->RunOnce(
//         [this]
//         {
//             m_algae_intake->MoveWristToCommand()
//         }
//     );
// }

frc2::CommandPtr t34::Coordinator::RunElevator(double speed)
{
    return m_coral_intake->MoveToZero().AndThen(m_elevator->MoveElevatorByPowerCommand(speed));
}

void t34::Coordinator::Periodic() 
{
    frc::SmartDashboard::PutNumber("Level", m_current_level);
}
