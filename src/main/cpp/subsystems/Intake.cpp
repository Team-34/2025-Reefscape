#include "subsystems/Intake.h"

namespace t34{
    
  Intake::Intake()
  : m_motor_1(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
  , m_motor_2(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {} 

  

  frc2::CommandPtr Intake::AlgaeInCommand(double power_percentage)
  {
    return this->StartEnd
    (
    [this, power_percentage] {
      this->m_motor_1.Set(power_percentage); 
      this->m_motor_2.Set(power_percentage);
    },
    [this, power_percentage] {
      this->m_motor_1.StopMotor();
      this->m_motor_2.StopMotor();
    }
    );
  }
  
  frc2::CommandPtr Intake::AlgaeOutCommand(double power_percentage)
  {
    return this->StartEnd(
      [this, power_percentage] {
        this->m_motor_1.Set(power_percentage);
        this->m_motor_2.Set(power_percentage);
      },
      [this, power_percentage] {
        this->m_motor_1.Set(0.0);
        this->m_motor_2.Set(0.0);
      }
    );
  }
}