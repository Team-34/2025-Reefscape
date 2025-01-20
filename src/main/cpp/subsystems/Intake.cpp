#include "subsystems/Intake.h"

namespace t34{
    
  Intake::Intake()
  : motor_1(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
  , motor_2(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {} 

  frc2::CommandPtr Intake::Move(double input)
  {
    motor_1.Set(input);
    motor_2.Set(input);
    return this->StartEnd([this]  {this->motor_1.Set(1.0), this->motor_2.Set(1.0); }, [this] { this->motor_1.Set(0.0), this->motor_2.Set(0.0); });   
  }
  
}