// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <units/length.h>
#include "Constants.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

using namespace ctre::phoenix6::hardware;
using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol::can;
namespace t34
{

  class Elevator : public frc2::SubsystemBase
  {
  public:

    Elevator();

    frc2::CommandPtr ElevateToCommand(units::inch_t height);
    frc2::CommandPtr MoveAlgaeWristToCommand(double enc_units); //units::degree_t angle
    frc2::CommandPtr MoveCoralWristToCommand(units::degree_t angle);

    frc2::CommandPtr MoveToLevelCommand(int level);
    frc2::CommandPtr MoveUpOnceCommand();
    frc2::CommandPtr MoveDownOnceCommand();
    frc2::CommandPtr MoveToRestCommand();

    frc2::CommandPtr MoveElevatorByPowerCommand(double val);
    frc2::CommandPtr MoveAlgaeWristByPowerCommand(double val);
    frc2::CommandPtr MoveCoralWristByPowerCommand(double val);

    frc2::CommandPtr MoveAlgaeWristByIncrementCommand(double increase);

    void Periodic() override;

  private:
    int m_level;

    const units::inch_t m_init_height;

    const units::degree_t m_init_algae_angle;
    const units::degree_t m_init_coral_angle;

    TalonSRX m_right_algae_wrist_motor;
    TalonSRX m_left_algae_wrist_motor;

    TalonSRX m_left_motor;
    TalonSRX m_right_motor;

    SparkMax m_coral_wrist_motor;

    frc::PIDController m_elevator_motors_pid;
    frc::PIDController m_algae_wrist_pid;
    frc::PIDController m_coral_wrist_pid;
  };

} // namespace t34
