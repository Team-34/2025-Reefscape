// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AlgaeIntake.h"
#include "units/angle.h"

class MoveAlgaeWristToCommand
    : public frc2::CommandHelper<frc2::Command, MoveAlgaeWristToCommand> {
 public:
  
  MoveAlgaeWristToCommand(t34::AlgaeIntake *intake, units::degree_t angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  t34::AlgaeIntake *m_intake;

  units::degree_t m_angle;
};
