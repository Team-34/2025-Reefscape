// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/controller/PIDController.h>
#include <units/length.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include "Constants.h"

using namespace ctre::phoenix::motorcontrol::can;
using namespace rev::spark;

class Elevator : public frc2::SubsystemBase {
public:
  Elevator();

  frc2::CommandPtr ElevateCommand(units::inch_t height);
  frc2::CommandPtr MoveWristToCommand(units::degree_t degree);
  frc2::CommandPtr MoveUpOnceCommand();
  frc2::CommandPtr MoveDownOnceCommand();
  frc2::CommandPtr MoveToLevelCommand();

private:
  int level;

  SparkMax m_wrist_motor;

  VictorSPX m_vert_motor_left;
  VictorSPX m_vert_motor_right;

  frc::PIDController m_vert_motors_PID;
  frc::PIDController m_wrist_motor_PID;
};