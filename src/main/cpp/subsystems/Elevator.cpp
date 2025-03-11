#include <algorithm>
#include <array>
#include <units/length.h>
#include <units/angle.h>
#include <utility>

#include "subsystems/Elevator.h"
#include "Neo.h"
#include "Talon.h"

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
  , m_init_height(18_in)
    //The wrists' angles are from 0 to 180 degrees (0 is straight down, and 180 is straight up, and 90 is parallel to the floor)
  , m_init_algae_angle(155_deg) //65 degrees away from horizontal
  , m_init_coral_angle(0_deg) 
  {
    m_elevator_motors_pid.SetTolerance(Neo::LengthToNEOUnit(0.5_in));
    m_right_motor.Follow(m_left_motor);
  }

  frc2::CommandPtr Elevator::MoveWristToCommand(WristType wrist, units::degree_t angle)
  {
    SparkMax* wrist_motor{ nullptr };

    if(wrist == WristType::kAlgae) //set wrist_motor to the right motor
    {
      wrist_motor = &m_algae_wrist_motor;

      m_wrist_motors_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * Neo::AngleToNEOUnit(angle - m_init_algae_angle));
    }
    else if(wrist == WristType::kCoral)
    {
      wrist_motor = &m_coral_wrist_motor;

      m_wrist_motors_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * Neo::AngleToNEOUnit(angle - m_init_coral_angle));
    }
    else
    {
      this->GetCurrentCommand()->Cancel();
    }

    return this->RunEnd(
      [this, angle, wrist_motor]
      {
        //Run wrist motor in respect to the setpoint
        wrist_motor->Set(m_wrist_motors_pid.Calculate(wrist_motor->GetEncoder().GetPosition()));
      },
      [this, wrist_motor]
      {
        //When finished, stop the wrist motor and delete the wrist_motor pointer
        wrist_motor->StopMotor();
      })
      .Until([this] { return m_wrist_motors_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::ElevateToCommand(units::inch_t height)
  {
    m_elevator_motors_pid.SetSetpoint(ELEVATOR_WINCH_GEAR_RATIO * Talon::LengthToSRXUnit(height - m_init_height));

    return this->RunEnd(
      [this, height]
      {
        //Run the elevator in respect to the given height
        m_left_motor.Set(
          ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          m_elevator_motors_pid.Calculate(m_left_motor.GetSelectedSensorPosition())
        );
      },
      [this]
      {
        // stop motor
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] { return m_elevator_motors_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::MoveToLevelCommand(int level)
  {                               //  Coral Intake     Elevator height
    static const std::array<std::pair<units::degree_t, units::inch_t>, 5> presets {{
      { 85_deg, 0_in },
      { 35_deg, 18_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
      { 35_deg, 31.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
      { 35_deg, 47.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
      { 88_deg, 72_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
    }};

    m_level = std::clamp(level, 0, static_cast <int> (presets.size()) - 1);
    
    auto [angle, height] = presets.at(level);

    return ElevateToCommand(height).AndThen(MoveWristToCommand(WristType::kCoral, angle));
  };

  frc2::CommandPtr Elevator::MoveUpOnceCommand()
  {
    return this->MoveToLevelCommand(m_level + 1); 
  }

  frc2::CommandPtr Elevator::MoveDownOnceCommand()
  {
    return this->MoveToLevelCommand(m_level - 1); 
  }

  frc2::CommandPtr Elevator::MoveToRestCommand()
  {
    //move 18 inches from start to provide space, and then move wrist.
    return this->ElevateToCommand(18_in).AndThen(MoveWristToCommand(WristType::kAlgae, 0_deg));
  }

  frc2::CommandPtr Elevator::MoveElevatorByPowerCommand(double val)
  {
    return this->RunOnce
    (
      [this, val]
      {
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
      }
    );
  }

  frc2::CommandPtr Elevator::MoveAlgaeWristByPowerCommand(double val)
  {
    return this->RunOnce([this, val] { m_algae_wrist_motor.Set(val); });
  }

  frc2::CommandPtr Elevator::MoveCoralWristByPowerCommand(double val)
  {
    return this->RunOnce([this, val] { m_coral_wrist_motor.Set(val); });
  }

} // namespace t34