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
  , m_init_algae_angle(155_deg) //65 degrees away from horizontal
  , m_init_coral_angle(0_deg) 

  , m_right_algae_wrist_motor(1)
  , m_left_algae_wrist_motor(2)

  , m_left_motor(11)
  , m_right_motor(12)

  , m_coral_wrist_motor(3, SparkLowLevel::MotorType::kBrushless)

  , m_elevator_motors_pid(0.5, 0.0, 0.0)
  , m_algae_wrist_pid(0.5, 0.0, 0.0)
  , m_coral_wrist_pid(0.25, 0.0, 0.0)
  {
    m_elevator_motors_pid.SetTolerance(Neo::LengthTo550Unit(0.5_in));
    m_coral_wrist_pid.SetTolerance( (0.25_deg) / 1_tr );
    m_algae_wrist_pid.SetTolerance( (0.25_deg) / 1_tr);
  }

  frc2::CommandPtr Elevator::MoveAlgaeWristToCommand(double encoder_units)
  {                                             //units::degree_t angle
    //std::clamp(angle, 0_deg, 115_deg);
    
    double clamped_encoder_units = std::clamp(encoder_units, -9000.0, 593.0);
    //m_algae_wrist_pid.SetSetpoint(BOTH_WRIST_GEAR_RATIO * ((angle - m_init_algae_angle) / 1_tr));
    m_algae_wrist_pid.SetSetpoint(clamped_encoder_units);
                      //BOTH_WRIST_GEAR_RATIO * ((angle - m_init_algae_angle) / 1_tr)

    return this->RunEnd(
      [this, clamped_encoder_units]
      {
        m_left_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_algae_wrist_pid.Calculate(m_left_algae_wrist_motor.GetSelectedSensorPosition()));
        m_right_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_algae_wrist_pid.Calculate(m_right_algae_wrist_motor.GetSelectedSensorPosition()));
      },
      [this]
      {
        m_left_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_right_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] 
      { 
        return m_algae_wrist_pid.AtSetpoint();
      });
  }

  frc2::CommandPtr Elevator::MoveAlgaeWristByIncrementCommand(double increase) {
    
    return this->RunEnd(
      [this, increase]
      {
        m_right_algae_wrist_motor.Set(
          ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          m_algae_wrist_pid.Calculate(
            m_right_algae_wrist_motor.GetSelectedSensorPosition(), 
            (m_algae_wrist_pid.GetSetpoint() + increase)));

        m_left_algae_wrist_motor.Set(
        ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
          m_algae_wrist_pid.Calculate(
            m_left_algae_wrist_motor.GetSelectedSensorPosition(), 
            (m_algae_wrist_pid.GetSetpoint() + increase)));
      },
      [this]
      {
          m_left_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
          m_right_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      })
      .Until([this] 
      { 
        return m_algae_wrist_pid.AtSetpoint();
      });
  }

frc2::CommandPtr Elevator::MoveCoralWristToCommand(units::degree_t angle)
  {
    std::clamp(angle, 0_deg, 150_deg);
    m_coral_wrist_pid.SetSetpoint(-426.755 + (0.0438 * (angle.value() - m_init_algae_angle.value())));//-(angle.value() - m_init_algae_angle.value()) * 11.923);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));

    return this->RunEnd(
      [this, angle]
      {
        m_coral_wrist_motor.Set(m_coral_wrist_pid.Calculate(m_coral_wrist_motor.GetEncoder().GetPosition()));
      },
      [this]
      {
          m_coral_wrist_motor.StopMotor();
      })
      .Until([this] 
      { 
        return m_coral_wrist_pid.AtSetpoint();
      });
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

    return ElevateToCommand(height).AndThen(MoveCoralWristToCommand(angle));
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
    //move 16.5 inches from start to provide space, and then move wrist.
    //return this->ElevateToCommand(m_init_height).AndThen(MoveAlgaeWristToCommand(0_deg));
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

  frc2::CommandPtr Elevator::MoveAlgaeWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
        { 
          m_left_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
          m_right_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -val); 
        },
        [this]
        {
          m_left_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
          m_right_algae_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0); 
        }
    );
  }

  frc2::CommandPtr Elevator::MoveCoralWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
      { 
        m_coral_wrist_motor.Set(-val);  
      },
      [this]
      {
        m_coral_wrist_motor.Set(0.0);
      }
    );
  }

  void Elevator::Periodic() {
      frc::SmartDashboard::PutNumber("Coral Wrist Encoder", m_coral_wrist_motor.GetEncoder().GetPosition());
      frc::SmartDashboard::PutNumber("Left Algae Wrist Encoder", m_left_algae_wrist_motor.GetSelectedSensorPosition());
      frc::SmartDashboard::PutNumber("Right Algae Wrist Encoder", m_right_algae_wrist_motor.GetSelectedSensorPosition());
  }

} // namespace t34