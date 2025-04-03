// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  , m_auto_leave(swerve_drive, 0_in, -3_ft, 0_deg)
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  ctrl->RightTrigger(0.75).WhileTrue(m_climber.RunArm(0.1));
  ctrl->LeftTrigger(0.75).WhileTrue(m_climber.RunArm(-0.1));
  ctrl->LeftBumper().WhileTrue(m_algae_intake.MoveWristByPowerCommand(-0.5));
  ctrl->RightBumper().WhileTrue(m_algae_intake.MoveWristByPowerCommand(0.5));
  //ctrl->POVRight().OnTrue(m_coral_intake.MoveWristToCommand(12.0));
  //ctrl->POVLeft().OnTrue(m_coral_intake.MoveWristToCommand(0));
  ctrl->POVUp().WhileTrue(m_elevator.MoveElevatorByPowerCommand(0.5));
  ctrl->POVDown().WhileTrue(m_elevator.MoveElevatorByPowerCommand(-0.5));
  ctrl->Back().OnTrue(swerve_drive->ZeroYawCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::RunEnd([this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, 0.3_m), 0.0);
  },
  [this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, 0_m), 0.0);
  });
}