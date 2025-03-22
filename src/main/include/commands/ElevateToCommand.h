// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "subsystems/Elevator.h"


class ElevateToCommand
    : public frc2::CommandHelper<frc2::Command, ElevateToCommand> {
 public:
  ElevateToCommand(t34::Elevator *elevator, units::inch_t height);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  t34::Elevator *m_elevator;

  units::inch_t m_height;

};
