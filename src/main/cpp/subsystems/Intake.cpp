#include "subsystems/Intake.h"

namespace t34{
    
  Intake::Intake()
  : motor_1(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
  , motor_2(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {} 

  frc2::InstantCommand Intake::RunMotors(double power_percentage)
  {
    return frc2::InstantCommand([this]{
      this->motor_1.Set(0.2); this->motor_2.Set(0.2);
      this->motor_1.Set(0.0), this->motor_2.Set(0.0);
    });
  }
  
}