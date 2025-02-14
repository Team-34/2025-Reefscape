#include "subsystems/CoralIntake.h"

namespace t34
{    
  CoralIntake::CoralIntake()
    : m_motor(30, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {
  } 

  frc2::CommandPtr CoralIntake::RunInCommand()
  {
    return this->StartEnd(
      [this] { this->m_motor.Set(-0.25); },
      [this] { this->m_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr CoralIntake::RunOutCommand()
  {
    return this->StartEnd(
      [this] { this->m_motor.Set(0.5); },
      [this] { this->m_motor.StopMotor(); }
    );
  }
}