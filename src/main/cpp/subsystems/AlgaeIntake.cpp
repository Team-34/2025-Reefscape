#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_intake_motor(5)
    , m_right_wrist_motor(1)
    , m_left_wrist_motor(2) //This motor has an encoder, the right does not
    , m_encoder_units(m_left_wrist_motor.GetSelectedSensorPosition())
    , m_setpoint(m_left_wrist_motor.GetSelectedSensorPosition())
    , m_sensor(3)
  {
    m_left_wrist_motor.Config_kP(0, 0.05);
    m_left_wrist_motor.Config_kI(0, 0.0);
    m_left_wrist_motor.Config_kD(0, 0.02);

    //m_left_wrist_motor.SetInverted(true);
    m_left_wrist_motor.ConfigClosedloopRamp(0.3);
    m_left_wrist_motor.SetSensorPhase(true);

    m_right_wrist_motor.Follow(m_left_wrist_motor);
    m_right_wrist_motor.SetInverted(true);

    frc::SmartDashboard::PutNumber("Raw Algae Wrist Setpoint", 0.0);
    

    //m_pid.SetSetpoint(m_left_wrist_motor.GetSelectedSensorPosition());
  } 

  void AlgaeIntake::MoveWristTo(int setpoint)
  {
    //m_setpoint = std::clamp(setpoint, 0.0, 8000.0); // Greater than ~8800 causes this wrist to hit the coral wrist
    frc::SmartDashboard::PutNumber("Raw Algae Wrist Setpoint", setpoint);

    m_setpoint = std::clamp(setpoint, -2120, 9000); // -2120 is the lower limit physically

    m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_setpoint);

    //m_pid.SetSetpoint(setpoint);
  }

  void AlgaeIntake::MoveWristTo(units::degree_t angle)
  {
    //m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_left_wrist_motor.GetSelectedSensorPosition(), angle.value()));
    //m_right_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, m_pid.Calculate(m_right_wrist_motor.GetSelectedSensorPosition(), angle.value()));
  }

  // frc2::CommandPtr AlgaeIntake::MoveWristToCommand(units::degree_t angle)
  // {
    
  // }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(int setpoint)
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
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, val);
      },
      [this]
      {
        m_left_wrist_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      } 
    );
  }

  frc2::CommandPtr AlgaeIntake::RunInCommand(double speed)
  {
    return this->RunEnd
    (
      [this, speed]
      {
        if (m_sensor.Get()){
          m_intake_motor.Set(-speed);
        } 
        else
        {
          m_intake_motor.Set(0.0);
        }

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
    //m_encoder_units = m_left_wrist_motor.GetSelectedSensorPosition();

    frc::SmartDashboard::PutNumber("Left Algae Wrist Motor Encoder", m_left_wrist_motor.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Actual Algae Wrist Setpoint", m_left_wrist_motor.GetClosedLoopTarget());
    frc::SmartDashboard::PutNumber("Clamped Algae Wrist Setpoint", m_setpoint);
    frc::SmartDashboard::PutNumber("Clamped Inverted Algae Wrist Setpoint", -m_setpoint);
    frc::SmartDashboard::PutBoolean("Algae Confirm", !m_sensor.Get());


    //double output = m_pid.Calculate(m_encoder_units);
  }
}