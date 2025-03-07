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
  , m_algae_wrist_motor(997, SparkLowLevel::MotorType::kBrushless)
  , m_coral_wrist_motor(998, SparkLowLevel::MotorType::kBrushless)
  , m_left_motor(999)
  , m_right_motor(1000)
  , m_elevator_motors_pid(0.5, 0.0, 0.0)
  , m_wrist_motors_pid(0.5, 0.0, 0.0)
  {
    m_elevator_motors_pid.SetTolerance(NEOUnitToInch(0.5));
    m_right_motor.Follow(m_left_motor);
  }

  frc2::CommandPtr Elevator::MoveWristToCommand(WristType wrist, units::degree_t angle)
  {
    m_wrist_motors_pid.SetSetpoint(NEOUnitToDegree(angle.value()));

    SparkMax* wrist_motor;

    if(wrist == WristType::kAlgae) //set wrist_motor to the right motor
    {
      wrist_motor = &m_algae_wrist_motor;
    }
    else if(wrist == WristType::kCoral)
    {
      wrist_motor = &m_coral_wrist_motor;
    }
    else
    {
      //if the enum is invalid, delete the wrist_motor pointer and end the command
      delete wrist_motor;
      return;
    }

    return this->RunEnd(
      [this, angle, wrist_motor]
      {
        //Run wrist motor in respect to the setpoint
        wrist_motor->Set(m_wrist_motors_pid.Calculate(NEOUnitToDegree(wrist_motor->GetEncoder().GetPosition())));
      },
      [this, wrist_motor]
      {
        //When finished, stop the wrist motor and delete the wrist_motor pointer
        wrist_motor->StopMotor();
        delete wrist_motor;
      })
      .Until([this] { return m_wrist_motors_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::ElevateToCommand(units::inch_t height)
  {
    m_elevator_motors_pid.SetSetpoint(NEOUnitToInch(height.value()));

    return this->RunEnd(
      [this, height]
      {
        m_left_motor.Set(
          ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          m_elevator_motors_pid.Calculate(
            NEOUnitToInch(m_left_motor.GetSelectedSensorPosition()),
            NEOUnitToInch(height.value())));
      },
      [this]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] { return m_elevator_motors_pid.AtSetpoint(); });
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
