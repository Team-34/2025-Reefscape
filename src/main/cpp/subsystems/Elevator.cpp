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

  , m_init_height(16.5_in)

    //The wrists' angles are from 0 to 180 degrees (0 is straight down, 180 is straight up, and 90 is parallel to the floor)
  , m_init_algae_angle(155_deg) //65 degrees away from horizontal
  , m_init_coral_angle(0_deg) 

  , m_right_algae_wrist_motor(1, SparkLowLevel::MotorType::kBrushless)
  , m_left_algae_wrist_motor(2, SparkLowLevel::MotorType::kBrushless)
  , m_coral_wrist_motor(3, SparkLowLevel::MotorType::kBrushless)

  , m_left_motor(11)
  , m_right_motor(12)

  , m_elevator_motors_pid(0.5, 0.0, 0.0)
  , m_right_algae_wrist_pid(0.5, 0.0, 0.0)
  , m_left_algae_wrist_pid(0.5, 0.0, 0.0)
  , m_coral_wrist_pid(0.5, 0.0, 0.0)
  {
    m_elevator_motors_pid.SetTolerance(Neo::LengthTo550Unit(0.5_in));
    m_coral_wrist_pid.SetTolerance(Neo::AngleTo550Unit(0.25_deg));
    m_left_algae_wrist_pid.SetTolerance(Neo::AngleTo550Unit(0.25_deg));
    m_right_algae_wrist_pid.SetTolerance(Neo::AngleTo550Unit(0.25_deg));
  }

  frc2::CommandPtr Elevator::MoveWristToCommand(WristType wrist, units::degree_t angle)
  {
    //SparkMax* wrist_motor{ nullptr };
    switch (wrist)
    {
    case(WristType::kAlgae): //set wrist_motor to the right motor
      //wrist_motor = &m_algae_wrist_motor;
      m_left_algae_wrist_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * Neo::AngleTo550Unit(angle - m_init_algae_angle));
      m_right_algae_wrist_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * Neo::AngleTo550Unit(angle - m_init_algae_angle));
      break;
    
    case(WristType::kCoral):
      //wrist_motor = &m_coral_wrist_motor;
      m_coral_wrist_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * Neo::AngleTo550Unit(angle - m_init_coral_angle));
      break;

    default:
      this->GetCurrentCommand()->Cancel();
      break;
    }

    return this->RunEnd(
      [this, angle, wrist]
      {
        //Run wrist motor in respect to the setpoint
        //wrist_motor->Set(m_wrist_motors_pid.Calculate(wrist_motor->GetEncoder().GetPosition()));
        switch (wrist)
        {
        case (WristType::kAlgae):
          m_left_algae_wrist_motor.Set(m_left_algae_wrist_pid.Calculate(m_left_algae_wrist_motor.GetEncoder().GetPosition()));
          m_right_algae_wrist_motor.Set(m_right_algae_wrist_pid.Calculate(m_right_algae_wrist_motor.GetEncoder().GetPosition()));
          break;
        
        case (WristType::kCoral):
          m_coral_wrist_motor.Set(m_coral_wrist_pid.Calculate(m_left_algae_wrist_motor.GetEncoder().GetPosition()));
          break;
        }
      },
      [this, wrist]
      {
        //When finished, stop the wrist motor
        //wrist_motor->StopMotor();
        switch (wrist)
        {
        case (WristType::kAlgae):
          m_left_algae_wrist_motor.StopMotor();
          m_right_algae_wrist_motor.StopMotor();
          break;
        
        case (WristType::kCoral):
          m_coral_wrist_motor.StopMotor();
          break;
        }
      })
      .Until([this, wrist] 
      { 
        switch (wrist)
        {
        case (WristType::kAlgae):
          return m_left_algae_wrist_pid.AtSetpoint() && m_right_algae_wrist_pid.AtSetpoint();
        
        case (WristType::kCoral):
          return m_coral_wrist_pid.AtSetpoint();

        default:
        return true;
        }
      });
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
  {                               //  Algae Intake     Elevator height
    static const std::array<std::pair<units::degree_t, units::inch_t>, 5> presets {{
      { 5_deg, 0_in },
      { 55_deg, 18_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 55_deg, 31.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 55_deg, 47.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 2_deg, 72_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
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
    return this->ElevateToCommand(m_init_height).AndThen(MoveWristToCommand(WristType::kAlgae, 0_deg));
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
    return this->RunOnce([this, val] 
    { 
      m_left_algae_wrist_motor.Set(val);
      m_right_algae_wrist_motor.Set(val); 
    });
  }

  frc2::CommandPtr Elevator::MoveCoralWristByPowerCommand(double val)
  {
    return this->RunOnce([this, val] { m_coral_wrist_motor.Set(val); });
  }

} // namespace t34