#include "RobotContainer.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/CoralIntake.h"

RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  , ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(swerve_drive, ctrl)
  , m_algae_intake()
  , m_coral_intake()
  , m_climber()
  , m_elevator()
  , m_coordinator(&m_elevator, &m_coral_intake)
  , m_auto_leave(swerve_drive, 0_in, -3_ft, 0_deg)
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  ctrl->RightTrigger(0.75).OnTrue(m_climber.Climb());

  ctrl->LeftTrigger(0.75)
    .OnTrue(m_elevator.HalfSpeed()
    .AlongWith(swerve_drive->ToggleFarisModeCommand()));

  ctrl->RightBumper().WhileTrue(m_algae_intake.MoveWristByPowerCommand(0.5));

  ctrl->LeftBumper().WhileTrue(m_algae_intake.MoveWristByPowerCommand(-0.5));

  (ctrl->POVRight() && !ctrl->LeftStick())
    .OnTrue(m_coral_intake.MoveWristToCommand(12.0));

  (ctrl->POVLeft() && !ctrl->LeftStick())
    .OnTrue(m_coral_intake.MoveToZero());

  (ctrl->POVDown() && !ctrl->LeftStick())
    .WhileTrue(m_elevator.MoveElevatorByPowerCommand(-0.8));

  (ctrl->POVUp() && !ctrl->LeftStick())
    .WhileTrue(m_elevator.MoveElevatorByPowerCommand(0.8));

  ctrl->Back().OnTrue(swerve_drive->ZeroYawCommand());

  ctrl->A().WhileTrue(m_algae_intake.RunInCommand(0.5));
  ctrl->B().WhileTrue(m_algae_intake.RunOutCommand(0.5));
  
  ctrl->X().WhileTrue(m_coral_intake.RunInCommand(0.5));
  ctrl->Y().WhileTrue(m_coral_intake.RunInCommand(0.5));  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return frc2::cmd::RunEnd([this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, -1.0_m), 0.0);
  },
  [this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, 0_m), 0.0);
  });
}