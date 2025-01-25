// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Command.h>

#include "commands/AutoDriveCommand.h"
#include "commands/CenterOnCoral.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"

class AlgaeAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AlgaeAutoCommand> {
  public:
  AlgaeAutoCommand(t34::SwerveDrive * swerve, Elevator * elevator, t34::Intake * intake);

};
