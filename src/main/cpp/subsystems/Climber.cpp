#include "subsystems/Climber.h"
#include "Constants.h"

Climber::Climber() 
: m_pid_controller(0.5, 0.5, 0.5)
, m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
{}

frc2::CommandPtr Climber::FlipArmUp() 
{
    return this->StartEnd(
        [this] {
            m_pid_controller.SetSetpoint(NEOUnitToDegree(90));
        },
        [this] {
           m_motor.Set(0.0); 
        }
    ).Until(
        [this] {
            return m_pid_controller.AtSetpoint();
        }
    );
}