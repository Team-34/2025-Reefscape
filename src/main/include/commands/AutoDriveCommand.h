// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/controller/PIDController.h>

#include "subsystems/SwerveDrive.h"


class AutoDriveCommand
    : public frc2::CommandHelper<frc2::Command, AutoDriveCommand> {
 public:

  AutoDriveCommand(t34::SwerveDrive * swerve, units::foot_t x_translation, units::foot_t y_translation, units::degree_t rotation);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  t34::SwerveDrive * m_swerve;
  units::foot_t m_x_translation;
  units::foot_t m_y_translation;
  units::degree_t m_rotation;

  frc::PIDController m_x_PID;
  frc::PIDController m_y_PID;

  double m_x_init_dist;
  double m_y_init_dist;
};
