#include "subsystems/AlgaeIntake.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_primary_motor(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
    , m_secondary_motor(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {
    SparkMaxConfig secondary_motor_config;
    secondary_motor_config.Follow(m_primary_motor);
    m_secondary_motor.Configure(
      secondary_motor_config,
      rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters
    );
  } 

  frc2::CommandPtr AlgaeIntake::RunInCommand()
  {
    return this->StartEnd(
      [this] { this->m_primary_motor.Set(-0.25); },
      [this] { this->m_primary_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr AlgaeIntake::RunOutCommand()
  {
    return this->StartEnd(
      [this] { this->m_primary_motor.Set(0.7); },
      [this] { this->m_primary_motor.StopMotor(); }
    );
  }
}