// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include "T34CommandXboxController.h"
#include "commands/ControllerDriveCommand.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/Climber.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"


using namespace t34;

class RobotContainer
{
public:
  RobotContainer();
  std::shared_ptr<t34::SwerveDrive> m_swerve;
  std::shared_ptr<t34::T34CommandXboxController> m_ctrl;
  
  t34::ControllerDriveCommand m_default_command;
  AlgaeIntake m_al_intake;
  Climber m_climber;
  CoralIntake m_co_intake;
  Elevator m_elevator;
  frc2::CommandPtr GetAutonomousCommand();

private:
  
  void ConfigureBindings();
};
