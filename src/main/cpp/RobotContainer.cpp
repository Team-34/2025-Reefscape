// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  , ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(swerve_drive, ctrl)
  
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  ctrl->A().WhileTrue(m_al_intake.RunInCommand());
  ctrl->B().WhileTrue(m_al_intake.RunOutCommand());
  //ctrl->POVUp().WhileTrue(m_elevator.ElevateToCommand(3_in));
  //ctrl->POVDown().WhileTrue(m_elevator.ElevateToCommand(-3_in));
  ctrl->POVUp().WhileTrue(m_elevator.MoveElevatorByPowerCommand(0.5));
  ctrl->POVDown().WhileTrue(m_elevator.MoveElevatorByPowerCommand(-0.5));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
