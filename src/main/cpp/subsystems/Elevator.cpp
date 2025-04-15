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
  : m_encoder(0)
  , m_level(0)
  , m_last_reading(0.0)
  , m_encoder_accumulation(0.0)
  , m_init_height(31.75_in) //height from the floor to the crossbar - the algae intake wheel is 4 in from the base
  , m_init_units(m_encoder.Get())
  , m_left_motor(11)
  , m_right_motor(12)
  , m_pid(1.0, 0.0, 0.15)
  , m_elevator_top_sensor(1)
  {

    m_pid.SetTolerance(0.2);

    TalonSRXConfiguration motor_config;

    motor_config.continuousCurrentLimit = 30;
    motor_config.peakCurrentDuration = 1500;
    motor_config.peakCurrentLimit = 40;
    motor_config.closedloopRamp = 0.25;
    motor_config.openloopRamp = 0.25;

    m_left_motor.ConfigAllSettings(motor_config);
    m_right_motor.ConfigAllSettings(motor_config);

    m_left_motor.ConfigClosedloopRamp(0.15);
    m_right_motor.ConfigClosedloopRamp(0.15);

    m_pid.SetSetpoint(m_encoder.Get());
  }

  void Elevator::ElevateTo(units::inch_t height)
  {
    m_pid.SetSetpoint((height.value() - m_init_height.value()) * 0.24);
  }

  void Elevator::ElevateTo(double height)
  {
    height = std::clamp(height, 0.0, 10.0);

    m_pid.SetSetpoint(height);
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
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }
    );
  }

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
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }
    );
  }

  frc2::CommandPtr Elevator::MoveElevatorByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val]
      {
        auto power = m_half_speed ? (val / 2) : val;
        frc::SmartDashboard::PutNumber("Elevator Power: ", power);
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power );
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -power );

        m_pid.SetSetpoint(m_encoder_accumulation);
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
    // Source: https://stackoverflow.com/a/50860805
    //
    //   int rotation_angle(int new_reading, int old_reading) {
    //     /* angle readings are in [0..360] range */
    //     /* compute the difference modulo 360 and shift it in range [-180..179] */
    //     return (360 + 180 + new_reading - old_reading) % 360 - 180;
    //   }
    //

    auto delta = fmod(1.0 + 0.5 + next - last, 1.0) - 0.5;
    return acc + delta;
  }

  units::inch_t Elevator::GetPosition()
  {
    return units::inch_t(GetPositionAsEncVal() * 1); //replace 1 with scalar to an inch
  }

  void Elevator::Periodic()
  {
    double next_reading = m_encoder.Get() - m_init_units;

    frc::SmartDashboard::PutNumber("Elevator Encoder Units with accum", m_encoder_accumulation);
    frc::SmartDashboard::PutNumber("Elevator Encoder Units", next_reading);
    frc::SmartDashboard::PutNumber("Elevator Setpoint", m_pid.GetSetpoint());
    frc::SmartDashboard::PutBoolean("Half Speed? ", m_half_speed);

    m_encoder_accumulation = UpdatePosition(m_encoder_accumulation, m_last_reading, next_reading);
    m_last_reading = next_reading;

    auto pid_output = m_pid.Calculate(m_encoder_accumulation);
    
    m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, pid_output);
    m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -pid_output);

    frc::SmartDashboard::PutNumber("Elevator PID Output", pid_output);
    frc::SmartDashboard::PutBoolean("Elev. Top Limit Switch", !m_elevator_top_sensor.Get());
    if (!m_elevator_top_sensor.Get())
    {
      m_encoder_accumulation = 10.0;
    }
  }
}
