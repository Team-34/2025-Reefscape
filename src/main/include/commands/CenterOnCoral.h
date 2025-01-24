// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <cmath>

#include "LimelightUtil.h"
#include "Robot.h"


class CenterOnCoral
    : public frc2::CommandHelper<frc2::Command, CenterOnCoral> {
 public:

  CenterOnCoral(t34::SwerveDrive * swerve_ptr);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


  private:

  frc::PIDController m_x_PID;
  frc::PIDController m_y_PID;

  t34::LimelightUtil m_LL;

  t34::SwerveDrive* m_swerve;

};
