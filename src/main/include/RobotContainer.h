// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "T34CommandXboxController.h"
#include "subsystems/SwerveDrive.h"
#include "commands/ControllerDriveCommand.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <memory>

class RobotContainer {
 public:
  RobotContainer();

  static RobotContainer* Get();

  std::shared_ptr<t34::SwerveDrive> swerve_drive;
  std::shared_ptr<t34::T34CommandXboxController> ctrl;

  t34::ControllerDriveCommand DefaultCommand;
  frc::SendableChooser<frc2::Command*> autoChooser;
  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();
};
