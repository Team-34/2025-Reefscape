#include "subsystems/Intake.h"

namespace t34
{    
  Intake::Intake()
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

  frc2::CommandPtr Intake::AlgaeInCommand(double power_percentage)
  {
    return this->StartEnd(
      [this, power_percentage] {
        this->m_primary_motor.Set(power_percentage); 
      },
      [this, power_percentage] {
        this->m_primary_motor.StopMotor();
      }
    );
  }
  
  frc2::CommandPtr Intake::AlgaeOutCommand(double power_percentage)
  {
    return this->StartEnd(
      [this, power_percentage] {
        this->m_primary_motor.Set(power_percentage);
      },
      [this, power_percentage] {
        this->m_primary_motor.StopMotor();
      }
    );
  }
}