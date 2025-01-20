// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/SwerveDrive.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <memory>
#include <iostream>

class RobotContainer {
 public:
  RobotContainer();

  static RobotContainer* Get();

  std::shared_ptr<t34::SwerveDrive> swerve_drive;

  frc::SendableChooser<frc2::Command*> autoChooser;
  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
};
