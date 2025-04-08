#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_wrist_motor(1)
    , m_left_wrist_motor(2) //This motor has encoder, the right does not
    , m_pid(0.2, 0.0, 0.05)
    , m_intake_motor(5)
  {
    m_left_wrist_motor.Config_kP(0, 0.2);
    m_left_wrist_motor.Config_kI(0, 0.0);
    m_left_wrist_motor.Config_kD(0, 0.05);

    m_pid.SetSetpoint(m_left_wrist_motor.GetSelectedSensorPosition());
  } 

  void AlgaeIntake::MoveWristTo(double setpoint)
  {
    setpoint = std::clamp(setpoint, -8500.0, 0.0);

    m_pid.SetSetpoint(setpoint);
  }

  void AlgaeIntake::MoveWristTo(units::degree_t angle)
  {
    //m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_left_wrist_motor.GetSelectedSensorPosition(), angle.value()));
    //m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_right_wrist_motor.GetSelectedSensorPosition(), angle.value()));

    //m_left_wrist_motor.GetClosedLoopController().SetReference(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    //m_right_wrist_motor.GetClosedLoopController().SetReference(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
  }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(units::degree_t angle)
  {
    return this->Run
    (
      [this, angle] 
        { 
        // m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, t34::Talon::AngleTo775ProUnit(angle));
        // m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_right_wrist_motor.GetSelectedSensorPosition(), angle.value()));
        }
    );
    //.Until(m_left_wrist_motor.GetSelectedSensorPosition() == angle.value()); // <-- UNTESTED
  }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(double setpoint)
  {
    return this->RunOnce([this, setpoint]
    {
      MoveWristTo(setpoint);
    });
  }

 frc2::CommandPtr AlgaeIntake::MoveWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
        { 
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -val);
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
        },
      [this]
        {
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        } 
    );
  }

  frc2::CommandPtr AlgaeIntake::MoveWristByIncrementCommand(double increase) 
  {
    return this->RunEnd
    (
      [this, increase]
      {
        double current_position = m_left_wrist_motor.GetSelectedSensorPosition();

        double new_setpoint = current_position + increase;
      },
      [this]
      {
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

      }
    );
  }

  frc2::CommandPtr AlgaeIntake::RunInCommand(double speed)
  {
    return this->RunEnd
    (
      [this, speed]
      {
        m_intake_motor.Set(speed);
      },
      [this]
      {
        m_intake_motor.Set(0.0);
      }
    );
  }

frc2::CommandPtr AlgaeIntake::RunOutCommand(double speed) 
{
  return this->RunEnd
  (
    [this, speed]
      {
        m_intake_motor.Set(speed);
      },
    [this]
      {
        m_intake_motor.Set(0.0);
      }
  );
}

  void AlgaeIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Left Algae Wrist Motor Encoder", m_left_wrist_motor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Algae Wrist Setpoint", m_pid.GetSetpoint());

    double output = m_pid.Calculate(m_left_wrist_motor.GetSelectedSensorPosition());
  }
}