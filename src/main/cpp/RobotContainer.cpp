// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

static std::unique_ptr<RobotContainer> g_rc{ nullptr };


RobotContainer::RobotContainer() 
  : ctrl(new t34::T34CommandXboxController(0))
  , swerve_drive(new t34::SwerveDrive())
  , m_default_command(swerve_drive, ctrl)
  , m_climber()
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 

{
//Runs algae intake in on A button, spits out on B
  ctrl->A().OnTrue(intake.AlgaeInCommand(-0.25));
  ctrl->B().OnTrue(intake.AlgaeOutCommand(0.7));

  //Moves the climber up and down on Left Bumper press
  ctrl->LeftBumper().OnTrue(m_climber.FlipArmCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
