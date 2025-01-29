// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

static std::unique_ptr<RobotContainer> g_rc{ nullptr };


RobotContainer* RobotContainer::Get() {
    if (!g_rc) {
        g_rc.reset(new RobotContainer());
    }

    return g_rc.get();
}

RobotContainer::RobotContainer() 
  : ctrl(new t34::T34CommandXboxController(0))
  , swerve_drive(new t34::SwerveDrive())
  , DefaultCommand(swerve_drive, ctrl)
  , intake()
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 

{
//Runs algae intake in on A button, spits out on B
  ctrl->A().OnTrue(intake.RunIn(-0.25));
  ctrl->B().OnTrue(intake.RunOut(0.7));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
