#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_motor(5)
    , m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_wrist_motor(1)
    , m_left_wrist_motor(2)
  {} 

  void AlgaeIntake::MoveWristTo(double enc_units)
  {
    m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, enc_units);
    m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, enc_units);
  }

  void AlgaeIntake::MoveWristTo(units::degree_t angle)
  {
    m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle.value());
    m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle.value());
  }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(units::degree_t angle)
  {
    return this->RunEnd
    (
      [this, angle] 
      { 
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle.value());
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, angle.value());
      },
      [this]
      {
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    );
  }

 frc2::CommandPtr AlgaeIntake::MoveWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
      { 
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
      },
      [this]
      {
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    );
  }
  void AlgaeIntake::Periodic() {}
}