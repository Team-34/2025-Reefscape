// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <units/length.h>
#include "Constants.h"


using namespace ctre::phoenix::motorcontrol::can;
using namespace rev::spark;

class Elevator : public frc2::SubsystemBase
{
public:
  Elevator();

  frc2::CommandPtr ElevateToCommand(units::inch_t height);
  frc2::CommandPtr MoveWristToCommand(units::degree_t angle);
  frc2::CommandPtr MoveUpOnceCommand();
  frc2::CommandPtr MoveDownOnceCommand();
  frc2::CommandPtr MoveToLevelCommand(int level);

private:
  int m_level;

  SparkMax m_wrist_motor;

  VictorSPX m_vertical_motor_left;
  VictorSPX m_vertical_motor_right;

  frc::PIDController m_vertical_motors_pid;
  frc::PIDController m_wrist_motor_pid;
};