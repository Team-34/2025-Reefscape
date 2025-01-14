// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/controller/PIDController.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
public:
  Elevator();

  
  void Periodic() override;

private:
  ctre::phoenix::motorcontrol::can::TalonSRX m_motors;

  frc::PIDController m_motors_PID;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
