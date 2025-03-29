#include "subsystems/CoralIntake.h"

namespace t34
{    
  CoralIntake::CoralIntake()
    : m_motor(4, rev::spark::SparkLowLevel::MotorType::kBrushless)
    , m_coral_level(0)
    , m_init_coral_angle(0_deg) 
    , m_wrist_motor(3, SparkLowLevel::MotorType::kBrushless)
    , m_run_up(false)
    , m_encoder_setpoint(0.0)
  {

    Register();
  } 

  frc2::CommandPtr CoralIntake::MoveWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
      { 
        m_wrist_motor.Set(-val);  
      },
      [this]
      {
        m_wrist_motor.Set(0.0);
      }
    );
  }

   frc2::CommandPtr CoralIntake::MoveToLevelCommand(int level)
  {
    static const std::array<double, 5> presets {{
      0.0, 4.0, 13.666, 19.83, 23.0
    }};

    m_coral_level = std::clamp(level, 0, static_cast<int>(presets.size()) - 1);

    // m_coral_level = std::clamp(level, 0, 4);

    // switch (m_coral_level)
    // {
    //   case 0:
    //     return MoveCoralWristToCommand(0.0);
    //     break;
    //   case 1:
    //     return MoveCoralWristToCommand(4.0);
    //     break;
    // }

    return MoveWristToCommand(presets.at(m_coral_level));
  }

   frc2::CommandPtr CoralIntake::IncrementDown()
  {
    return this->MoveToLevelCommand(m_coral_level + 1);  
  }

  frc2::CommandPtr CoralIntake::IncrementUp()
  {
    return this->MoveToLevelCommand(m_coral_level - 1); 
  }

  void CoralIntake::MoveWristTo(double enc_units)
  {
    m_encoder_setpoint = enc_units;

    m_run_up = m_wrist_motor.GetEncoder().GetPosition() < enc_units;

    //m_wrist_pid.SetSetpoint(encoder_units);//-(angle.value() - m_init_algae_angle.value()) * 11.923);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));
      m_wrist_motor.Set((m_run_up) ? 0.3 : -0.3);
  }

  void CoralIntake::StopWrist()
  {
    m_wrist_motor.StopMotor();
  }

  bool CoralIntake::EndCondition()
  {
    if (m_run_up)
    {
      return m_wrist_motor.GetEncoder().GetPosition() >= (m_encoder_setpoint - 0.1);
    }
    else
    {
      return m_wrist_motor.GetEncoder().GetPosition() <= (m_encoder_setpoint + 0.1);
    }
  }

 frc2::CommandPtr CoralIntake::MoveWristToCommand(double encoder_units)
  {

    //bool run_up = m_wrist_motor.GetEncoder().GetPosition() < encoder_units;
    //m_wrist_pid.SetSetpoint(encoder_units);//-(angle.value() - m_init_algae_angle.value()) * 11.923);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));
    

    // return this->RunEnd(
    //   [this, run_up]
    //   {

    //     m_wrist_motor.Set((run_up) ? 0.3 : -0.3);
    //   },
    //   [this]
    //   {
    //       m_wrist_motor.StopMotor();
    //   })
    //   .Until([this, encoder_units, run_up] 
    //   { 
    //     if (run_up)
    //     {
    //       return m_wrist_motor.GetEncoder().GetPosition() >= (encoder_units - 0.1);
    //     }
    //     else
    //     {
    //       return m_wrist_motor.GetEncoder().GetPosition() <= (encoder_units + 0.1);
    //     }
    //   });

    return this->RunOnce(
      [this, encoder_units]
      {
        m_encoder_setpoint = encoder_units;
      }
    );
  }

  frc2::CommandPtr CoralIntake::MoveWristToCommand(units::degree_t angle)
  {
    angle = std::clamp(angle, 45_deg, 158_deg);

    return this->RunOnce(
      [this, angle]
      {
        m_wrist_motor.GetClosedLoopController().SetReference((angle.value() - m_init_coral_angle.value()) / 7.45, rev::spark::SparkLowLevel::ControlType::kPosition);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));
      }
      );
  }

  frc2::CommandPtr CoralIntake::RunInCommand()
  {
    return this->RunEnd(
      [this] { this->m_motor.Set(-0.25); },
      [this] { this->m_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr CoralIntake::RunOutCommand()
  {
    return this->RunEnd(
      [this] { this->m_motor.Set(0.5); },
      [this] { this->m_motor.StopMotor(); }
    );
  }

 void CoralIntake::Periodic()
  {
    frc::SmartDashboard::PutNumber("Coral Wrist Encoder: ", m_wrist_motor.GetEncoder().GetPosition());

    m_wrist_motor.GetClosedLoopController().SetReference(m_encoder_setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);

  }
}