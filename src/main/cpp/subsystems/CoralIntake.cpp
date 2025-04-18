#include "subsystems/CoralIntake.h"
#include <array>

namespace t34
{    
  CoralIntake::CoralIntake()
    : m_top_limit(2)
    , m_motor(4, rev::spark::SparkLowLevel::MotorType::kBrushless)
    , m_coral_level(0)
    , m_init_coral_angle(0_deg) 
    , m_wrist_motor(3, SparkLowLevel::MotorType::kBrushless)
    , m_run_up(false)
    , m_returning(false)
    , m_encoder_setpoint(0.0)
  {
    m_config.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pid(0.02, 0.0, 0.05);

    m_config.ClosedLoopRampRate(0.75);

    m_wrist_motor.Configure(m_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    frc::SmartDashboard::PutNumber("Coral Wrist Encoder: ", m_wrist_motor.GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Coral Wrist Setpoint: ", m_encoder_setpoint);
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

    m_wrist_motor.Set((m_run_up) ? 0.3 : -0.3);
  }

 frc2::CommandPtr CoralIntake::MoveWristToCommand(double encoder_units)
  {
    return this->RunOnce(
      [this, encoder_units]
      {
        m_encoder_setpoint = encoder_units;
        frc::SmartDashboard::PutNumber("Coral Wrist Encoder: ", encoder_units);
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

  frc2::CommandPtr CoralIntake::RunInCommand(double speed)
  {
    return this->RunEnd(
      [this, speed] { this->m_motor.Set(speed);},
      [this] { this->m_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr CoralIntake::RunOutCommand(double speed)
  {
    return this->RunEnd(
      [this, speed] { this->m_motor.Set(-speed);},
      [this] { this->m_motor.StopMotor(); }
    );
  }

  frc2::CommandPtr CoralIntake::MoveToZero()
  {
    return this->RunEnd(
      [this]
      {
        m_wrist_motor.Set(-0.2);
        m_returning = true;
      }
      , [this]
      {
        m_wrist_motor.StopMotor();
        m_wrist_motor.GetEncoder().SetPosition(0.0);

        m_encoder_setpoint = 0.0;

        m_returning = false;
      }
    ).Until(
      [this]
      {
        return this->AtTopLimit();
    });
  }

  void CoralIntake::Periodic()
  {
    
    frc::SmartDashboard::PutBoolean("Coral Limit engaged?", AtTopLimit());
    frc::SmartDashboard::PutNumber("Coral Enc", m_wrist_motor.GetEncoder().GetPosition());
    if (!m_returning)
    {
      m_wrist_motor.GetClosedLoopController().SetReference(m_encoder_setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
    }
  }
}