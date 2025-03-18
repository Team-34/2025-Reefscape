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
  , m_init_height(16_in)
    //The wrists' angles are from 0 to 180 degrees (0 is straight down, 180 is straight up, and 90 is parallel to the floor)
  , m_left_motor(11)
  , m_right_motor(12)
  , m_elevator_motors_pid(0.5, 0.0, 0.0)
  {
    m_elevator_motors_pid.SetTolerance(Neo::LengthTo550Unit(0.5_in));
  }

  frc2::CommandPtr Elevator::ElevateToCommand(units::inch_t height)
  {
    m_elevator_motors_pid.SetSetpoint(ELEVATOR_WINCH_GEAR_RATIO * Talon::LengthTo775ProUnit(height - m_init_height));

    return this->RunEnd(
      [this, height]
      {

        auto speed = m_elevator_motors_pid.Calculate(m_left_motor.GetSelectedSensorPosition());

        //Run the elevator in respect to the given height
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);

        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -speed);
      },
      [this]
      {
        // stop motor
        m_left_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_right_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] { return m_elevator_motors_pid.AtSetpoint(); });
  }

  frc2::CommandPtr Elevator::MoveToLevelCommand(int level)
  {                               //  Coral Intake     Elevator height
    static const std::array<std::pair<units::degree_t, units::inch_t>, 5> presets {{
      { 5_deg, 3_in }, //The swerve modules are 2.75 inches above the elevator's lowest point. The 775Pros will collide with the modules if they are ran all the way to the bottom.
      { 55_deg, 18_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 55_deg, 31.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 55_deg, 47.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
      { 2_deg, 72_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE - m_init_height },
    }};

    m_level = std::clamp(level, 0, static_cast<int>(presets.size()) - 1);
    
    auto [angle, height] = presets.at(level);

    return ElevateToCommand(height).AndThen(m_coral_intake.MoveCoralWristToCommand(angle));
  };

  frc2::CommandPtr Elevator::MoveUpOnceCommand()
  {
    return this->MoveToLevelCommand(m_level + 1); 
  }

  frc2::CommandPtr Elevator::MoveDownOnceCommand()
  {
    return this->MoveToLevelCommand(m_level - 1); 
  }

  // frc2::CommandPtr Elevator::MoveToRestCommand()
  // {
  //   //move 16.5 inches from start to provide space, and then move wrist.
  //   return this->ElevateToCommand(m_init_height).AndThen(MoveAlgaeWristToCommand(0_deg));
  // }

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



     

} // namespace t34