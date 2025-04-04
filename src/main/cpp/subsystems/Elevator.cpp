#include <algorithm>
#include <array>
#include <units/length.h>
#include <units/angle.h>
#include <utility>
#include <algorithm>
#include "subsystems/Elevator.h"
#include "Neo.h"
#include "Talon.h"
#include <frc/smartdashboard/SmartDashboard.h>

namespace t34
{
  Elevator::Elevator()
  : m_level(0)
  , m_last_reading(0.0)
  , m_encoder_accumulation(0.0)
  , m_init_height(16_in)
    //The wrists' angles are from 0 to 180 degrees (0 is straight down, 180 is straight up, and 90 is parallel to the floor)
  , m_left_motor(11)
  , m_right_motor(12)
  , m_pid(0.5, 0.0, 0.0)
  , m_encoder(0)
  {
    m_pid.SetTolerance(0.2);

    TalonSRXConfiguration motor_config;

    motor_config.continuousCurrentLimit = 30;
    motor_config.peakCurrentDuration = 1500;
    motor_config.peakCurrentLimit = 40;

    m_left_motor.ConfigAllSettings(motor_config);
    m_right_motor.ConfigAllSettings(motor_config);

    m_pid.SetSetpoint(m_encoder.Get());
    
    Register();

  }

  void Elevator::ElevateTo(units::inch_t height)
  {
    m_pid.SetSetpoint(ELEVATOR_WINCH_GEAR_RATIO * Talon::LengthTo775ProUnit(height - m_init_height));
    m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_encoder_accumulation));
    m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_encoder_accumulation));

  }

  void Elevator::ElevateTo(double height)
  {
    m_pid.SetSetpoint(height);
    m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_encoder_accumulation, height));
    m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_encoder_accumulation, height));

  }

  void Elevator::Stop()
  {
    m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  }

  frc2::CommandPtr Elevator::ElevateToCommand(units::inch_t height)
  {
    return this->RunEnd(
      [this, height]
      {
        ElevateTo(height);
      }
      , [this]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }
    );

    // m_pid.SetSetpoint(height.value());

    // return this->RunEnd(
    //   [this, height]
    //   {
    //     auto pos = m_pid.Calculate(GetPosition().value());

    //     //Run the elevator in respect to the given height
    //     m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, pos);

    //     m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, -pos);
    //   },
    //   [this]
    //   {
    //     // stop motor
    //     m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    //     m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    //   })
    //   .Until([this] { return m_pid.AtSetpoint(); });
  }

  // frc2::CommandPtr Elevator::ElevateToEncValue(double enc_value) {
  //   return this->RunEnd(
  //     [this, enc_value]
  //     {
  //       if (m_encoder_accumulation > enc_value)
  //       {
  //         m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
  //         m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
  //       } else if (m_encoder_accumulation < enc_value)
  //       {
  //         m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.25);
  //         m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.25);
  //       }
  //     },
  //     [this]
  //     {
  //         m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  //         m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
  //     }
  //   );
  // }

  frc2::CommandPtr Elevator::ElevateToCommand(double height)
  {
    return this->RunEnd(
      [this, height]
      {
        ElevateTo(height);
      }
      , [this]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }
    );
  }

  frc2::CommandPtr Elevator::MoveElevatorByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -val);
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
      },
      [this]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }
    );
  }

  

  double Elevator::UpdatePosition(double acc, double last, double next)
  {
    if (0.75 < last && next < 0.25)
    {
	    acc += 1.0 + (next - last);
    }

    else if (0.75 < next && last < 0.25)
    {
	    acc -= 1.0 - (next - last);
    }

    else
    {
	    acc += next - last;
    }

    return acc;
  }

  units::inch_t Elevator::GetPosition()
  {
    return units::inch_t(GetPositionAsEncVal() * 1); //replace 1 with scalar to an inch
  }

  void Elevator::Periodic()
  {
    double next_reading = m_encoder.Get();

    frc::SmartDashboard::PutNumber("Elevator Encoder Units with accum", m_encoder_accumulation);
    frc::SmartDashboard::PutNumber("Elevator Encoder Units", next_reading);
    frc::SmartDashboard::PutNumber("Elevator Setpoint", m_pid.GetSetpoint());

    m_encoder_accumulation = UpdatePosition(m_encoder_accumulation, m_last_reading, next_reading);
    m_last_reading = next_reading;

    //double Nsetpoint = m_pid.GetSetpoint();

  }
}