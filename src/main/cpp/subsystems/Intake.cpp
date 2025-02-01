#include "subsystems/Intake.h"

namespace t34{
    
  Intake::Intake()
  : m_motor_1(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
  , m_motor_2(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {
    SparkMaxConfig motor_2_config;
    motor_2_config.Follow(m_motor_1);
    m_motor_2.Configure(
      motor_2_config,
      rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters
    );
  } 

  

  frc2::CommandPtr Intake::AlgaeInCommand(double power_percentage)
  {
    return this->StartEnd
    (
      [this, power_percentage] {
        this->m_motor_1.Set(power_percentage); 
      },
      [this, power_percentage] {
        this->m_motor_1.StopMotor();
      }
    );
  }
  
  frc2::CommandPtr Intake::AlgaeOutCommand(double power_percentage)
  {
    return this->StartEnd(
      [this, power_percentage] {
        this->m_motor_1.Set(power_percentage);
      },
      [this, power_percentage] {
        this->m_motor_1.StopMotor();
      }
    );
  }
}