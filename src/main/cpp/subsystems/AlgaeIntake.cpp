#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_motor(5)
    , m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_algae_wrist_motor(1)
    , m_left_algae_wrist_motor(2)
    , m_algae_wrist_pid(0.5, 0.0, 0.0)
  {
    m_algae_wrist_pid.SetTolerance( (0.25_deg) / 1_tr);
  } 

 frc2::CommandPtr AlgaeIntake::MoveAlgaeWristToCommand(double encoder_units)
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

  frc2::CommandPtr AlgaeIntake::RunInCommand()
  {
    return this->RunEnd(
      [this] { m_motor.Set(-0.25); },
      [this] { m_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr AlgaeIntake::RunOutCommand()
  {
    return this->RunEnd(
      [this] { m_motor.Set(0.7); },
      [this] { m_motor.StopMotor(); }
    );
  }

   frc2::CommandPtr AlgaeIntake::MoveAlgaeWristByIncrementCommand(double increase) {
    
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

 frc2::CommandPtr AlgaeIntake::MoveAlgaeWristByPowerCommand(double val)
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

  void AlgaeIntake::Periodic()
  {
    frc::SmartDashboard::PutNumber("Left Algae Wrist Encoder", m_left_algae_wrist_motor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Right Algae Wrist Encoder", m_right_algae_wrist_motor.GetSelectedSensorPosition());
  }


}