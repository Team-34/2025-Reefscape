// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/Intake.h"
#include "T34CommandXboxController.h"
#include "subsystems/SwerveDrive.h"
#include "commands/ControllerDriveCommand.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/Elevator.h"
class RobotContainer {
 public:
  RobotContainer();

  static RobotContainer* Get();
  CoralIntake coralintake;
  std::shared_ptr<t34::SwerveDrive> swerve_drive;
  std::shared_ptr<t34::T34CommandXboxController> ctrl;
  t34::Intake intake;
  t34::ControllerDriveCommand DefaultCommand;
  Elevator elevator;
  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
};
