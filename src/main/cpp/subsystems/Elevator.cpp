#include <algorithm>
#include <array>
#include <units/length.h>
#include <units/angle.h>
#include <utility>

#include "subsystems/Elevator.h"

namespace t34
{

  Elevator::Elevator()
  : m_level(0)
  , m_wrist_motor(998, SparkLowLevel::MotorType::kBrushless)
  , m_vertical_motor_left(999)
  , m_vertical_motor_right(1000)
  , m_vertical_motors_pid(0.5, 0.0, 0.0)
  , m_wrist_motor_pid(0.5, 0.0, 0.0)
  {
    m_vertical_motors_pid.SetTolerance(NEOUnitToInch(0.5));
    m_vertical_motor_right.Follow(m_vertical_motor_left);
  }

  frc2::CommandPtr Elevator::MoveWristToCommand(units::degree_t angle)
  {
    return this->RunEnd(
      [this, angle]
      {
        m_wrist_motor.Set(m_wrist_motor_pid.Calculate(
          NEOUnitToDegree(m_wrist_motor.GetEncoder().GetPosition()), NEOUnitToDegree(angle.value())));
      },
      [this]
      {
        m_wrist_motor.StopMotor();
      })
      .Until([this] { return m_wrist_motor_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::ElevateToCommand(units::inch_t height)
  {
    return this->RunEnd(
      [this, height]
      {
        m_vertical_motor_left.Set(
          ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          m_vertical_motors_pid.Calculate(
            NEOUnitToInch(m_vertical_motor_left.GetSelectedSensorPosition()),
            NEOUnitToInch(height.value())));
      },
      [this]
      {
        m_vertical_motor_left.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] { return m_vertical_motors_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::MoveToLevelCommand(int)
  {
    static const std::array<std::pair<units::degree_t, units::inch_t>, 5> presets {{
      { 85_deg, 0_in },
      { 35_deg, 18_in - BASE_HEIGHT_FROM_FLOOR },
      { 35_deg, 31.875_in - BASE_HEIGHT_FROM_FLOOR },
      { 35_deg, 47.875_in - BASE_HEIGHT_FROM_FLOOR },
      { 88_deg, 72_in - BASE_HEIGHT_FROM_FLOOR },
    }};

    m_level = std::clamp(m_level, 0, static_cast <int> (presets.size()) - 1);
    
    auto [angle, height] = presets.at(m_level);

    return MoveWristToCommand(angle).AndThen(ElevateToCommand(height));
  };


  frc2::CommandPtr Elevator::MoveUpOnceCommand()
  {
    return this->MoveToLevelCommand(m_level + 1); 
  }

  frc2::CommandPtr Elevator::MoveDownOnceCommand()
  {
    return this->MoveToLevelCommand(m_level - 1); 
  }

} // namespace t34
