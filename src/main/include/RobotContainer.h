// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "T34CommandXboxController.h"
#include "commands/ControllerDriveCommand.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/Climber.h"
#include "Subsystems/CoralIntake.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/SwerveDrive.h"

class RobotContainer
{
public:
  RobotContainer();

  std::shared_ptr<t34::SwerveDrive> swerve_drive;
  std::shared_ptr<t34::T34CommandXboxController> ctrl;
  t34::ControllerDriveCommand m_default_command;

  frc2::CommandPtr GetAutonomousCommand();

private:
  t34::AlgaeIntake m_algae_intake;
  Climber          m_climber;
  t34::CoralIntake m_coral_intake;
  t34::Elevator    m_elevator;
  
  void ConfigureBindings();
};
