// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  , ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(swerve_drive, ctrl)
  , m_climber()
  , m_algae_intake()
  , m_coral_intake()
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  //Runs algae intake in on A button, spits out on B
  ctrl->A().OnTrue(m_algae_intake.RunInCommand());
  ctrl->B().OnTrue(m_algae_intake.RunOutCommand());
  //It runs coral intake on X button and spits out on Y
  ctrl->X().OnTrue(m_coral_intake.RunInCommand());
  ctrl->Y().OnTrue(m_coral_intake.RunOutCommand());

  //Moves the climber up and down on Left Bumper press
  ctrl->LeftBumper().OnTrue(m_climber.FlipArmCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
