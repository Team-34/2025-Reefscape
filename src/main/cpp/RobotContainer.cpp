// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

static std::unique_ptr<RobotContainer> g_rc{ nullptr };



RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  //, DefaultCommand(swerve_drive, ctrl)
{
  ConfigureBindings();
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser("New Auto");
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

RobotContainer* RobotContainer::Get() {
    if (!g_rc) {
        g_rc.reset(new RobotContainer());
    }

    return g_rc.get();
    
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //auto myCommand{std::make_unique<frc2::InstantCommand>([this] { autoChooser.GetSelected(); })}; 
  //frc2::CommandPtr wrappedCommand(std::move(myCommand));
  //return wrappedCommand;

  return pathplanner::PathPlannerAuto("New Auto").ToPtr();
}

