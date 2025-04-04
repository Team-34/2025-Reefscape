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
    , m_encoder_setpoint(0.0)
  {
    Register();

    m_config
      .Inverted(false)
      .SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    m_config.encoder
      .PositionConversionFactor(1)//1000
      .VelocityConversionFactor(1);//1000
    m_config.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pid(0.3, 0.0, 0.0);

    m_wrist_motor.Configure(m_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
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

  void CoralIntake::Periodic()
  {
    frc::SmartDashboard::PutNumber("Coral Wrist Encoder: ", m_wrist_motor.GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Coral Wrist Setpoint: ", m_encoder_setpoint);

    // if (m_top_limit.Get() && m_encoder_setpoint < m_wrist_motor.GetEncoder().GetPosition())
    // {
    //   m_wrist_motor.StopMotor();
    // }
    // else
    // {
    //   m_wrist_motor.GetClosedLoopController().SetReference(m_encoder_setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
    // }
  }
}