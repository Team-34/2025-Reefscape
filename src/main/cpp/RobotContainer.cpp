// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  , ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(swerve_drive, ctrl)
  , m_algae_intake()
  , m_coral_intake()
  , m_climber()
  , m_elevator()
  , m_auto_leave(swerve_drive, 0_in, 7_ft, 0_deg)
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  // ctrl->A().WhileTrue(m_algae_intake.RunInCommand());
  // ctrl->B().WhileTrue(m_algae_intake.RunOutCommand());

  ctrl->X().WhileTrue(m_coral_intake.RunInCommand());
  ctrl->Y().WhileTrue(m_coral_intake.RunOutCommand());

  ctrl->POVUp().WhileTrue(m_elevator.MoveElevatorByPowerCommand(0.5));
  ctrl->POVDown().WhileTrue(m_elevator.MoveElevatorByPowerCommand(-0.5));

  ctrl->RightBumper().WhileTrue(m_elevator.MoveAlgaeWristByPowerCommand(0.25));
  ctrl->LeftBumper().WhileTrue(m_elevator.MoveAlgaeWristByPowerCommand(-0.25));

//  ctrl->RightBumper().WhileTrue(m_elevator.MoveAlgaeWristToCommand(90_deg));
//  ctrl->LeftBumper().WhileTrue(m_elevator.MoveAlgaeWristToCommand(135_deg));

  // ctrl->RightBumper().WhileTrue(m_elevator.MoveAlgaeWristByPowerCommand(0.25));
  // ctrl->LeftBumper().WhileTrue(m_elevator.MoveAlgaeWristByPowerCommand(-0.25));

  ctrl->POVRight().WhileTrue(m_elevator.MoveCoralWristByPowerCommand(0.2));
  ctrl->POVLeft().WhileTrue(m_elevator.MoveCoralWristByPowerCommand(-0.2));

  // ctrl->POVRight().WhileTrue(m_elevator.MoveCoralWristToCommand(90_deg));
  // ctrl->POVLeft().WhileTrue(m_elevator.MoveCoralWristToCommand(135_deg));

  ctrl->A().WhileTrue(m_climber.RunLock(0.65));
  ctrl->B().WhileTrue(m_climber.RunLock(0.0));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
