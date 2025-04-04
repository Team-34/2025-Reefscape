#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_setpoint(m_left_wrist_motor.GetSelectedSensorPosition())
    , m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_wrist_motor(1)
    , m_left_wrist_motor(2) //This motor has encoder, the right does not
    //, m_pid(0.2, 0.0, 0.05)
    , m_intake_motor(5)
  {
    m_left_wrist_motor.Config_kP(0, 0.2);
    m_left_wrist_motor.Config_kI(0, 0.0);
    m_left_wrist_motor.Config_kD(0, 0.05);
  } 

  void AlgaeIntake::MoveWristTo(double setpoint)
  {
    m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, setpoint);
    m_right_wrist_motor.Follow(m_left_wrist_motor);
    // m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_left_wrist_motor.GetSelectedSensorPosition(), setpoint));
    // m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_right_wrist_motor.GetSelectedSensorPosition(), setpoint));
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

        double new_setpoint = current_position + increase; // setting the setpoint as the right motor's positiion plus an input increase, may need to be reversed

        //m_pid.SetSetpoint(new_setpoint); 
        // m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_pid.Calculate(current_position, new_setpoint));
        // m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_pid.Calculate(current_position, new_setpoint));

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
        m_intake_motor.Set(-speed);
      },
    [this]
      {
        m_intake_motor.Set(0.0);
      }
  );
}

  void AlgaeIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Right Algae Wrist Motor Encoder", m_left_wrist_motor.GetSelectedSensorPosition());

    //m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_left_wrist_motor.GetSelectedSensorPosition(), m_setpoint));
    //m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_right_wrist_motor.GetSelectedSensorPosition(), m_setpoint));

  }
}