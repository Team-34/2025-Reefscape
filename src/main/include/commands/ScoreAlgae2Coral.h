// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/SwerveDrive.h>
#include <commands/AutoDriveCommand.h>
#include <commands/CenterOnATCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
class ScoreAlgae2Coral
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 ScoreAlgae2Coral> {
 public:
  ScoreAlgae2Coral(std::shared_ptr<t34::SwerveDrive> swerve);

  private:
  std::shared_ptr<t34::SwerveDrive> m_swerve;
};
