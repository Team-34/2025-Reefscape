// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/AutoScoreAlgaeAndCoralCommand.h"
#include "commands/AutoDriveCommand.h"
#include "commands/CenterOnCoral.h"
#include "subsystems/SwerveDrive.h"


class AutoScoreAlgaeAndCoralCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoScoreAlgaeAndCoralCommand> {
  public:
  AutoScoreAlgaeAndCoralCommand(std::shared_ptr<t34::SwerveDrive> swerve);

  private:
  std::shared_ptr<t34::SwerveDrive> m_swerve;
};
