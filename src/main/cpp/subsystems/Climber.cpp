#include "subsystems/Climber.h"
#include "Constants.h"

Climber::Climber() 
: m_pid_controller(0.5, 0.0, 0.1)
, m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_climber_up(false)
{}

frc2::CommandPtr Climber::FlipArmUp() {
    return this->StartEnd(
        [this] {
            //m_pid_controller.SetSetpoint(NEOUnitToDegree(90));
            m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), NEOUnitToDegree(90.0));
        },
        [this] {
           m_motor.Set(0.0); 
        }
    ).Until([this]{
        return m_pid_controller.AtSetpoint();
    });
}

frc2::CommandPtr Climber::FlipArmDown() {
    return this->StartEnd(
        [this] {
            //m_pid_controller.SetSetpoint(NEOUnitToDegree(0));
            m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), NEOUnitToDegree(90.0));
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