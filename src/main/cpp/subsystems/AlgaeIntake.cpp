#include "subsystems/AlgaeIntake.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_motor(5)
  {
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
}