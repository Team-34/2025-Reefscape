// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <frc/AnalogEncoder.h>

#include "Constants.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/AlgaeIntake.h"

using namespace ctre::phoenix::motorcontrol::can;

namespace t34
{

  class Elevator : public frc2::SubsystemBase
  {
  public:

    Elevator();

    frc2::CommandPtr ElevateToCommand(units::inch_t height);
    void ElevateTo(units::inch_t height);
    void Stop();

    units::inch_t GetPosition();

    inline bool EndCondition() { return m_elevator_motors_pid.AtSetpoint(); };

    //frc2::CommandPtr MoveToRestCommand();

    frc2::CommandPtr MoveElevatorByPowerCommand(double val);

    inline units::inch_t GetInitHeight() { return m_init_height; }    
    
    inline double GetPositionAsEncVal() { return m_encoder_accumulation * 360.0; }

    void Periodic() override;

  private:

    void UpdatePosition();

    int m_level;

    double m_last_reading;

    double m_encoder_accumulation; //The USDigital MA3 encoder gets a looped 0-1 value, so it's required to get the total units

    const units::inch_t m_init_height;

    TalonSRX m_left_motor;
    TalonSRX m_right_motor;

    frc::AnalogEncoder m_encoder;

    frc::PIDController m_elevator_motors_pid;
  };

} // namespace t34
