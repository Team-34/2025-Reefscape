#include "subsystems/Climber.h"

Climber::Climber() 
: m_pid_controller(0.1, 0.0, 0.0)
, m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_engaged(false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

frc2::CommandPtr Climber::FlipArmCommand()
{
    return this->RunEnd(
        [this] {
            if (m_engaged){
                m_motor.Set(m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), 0.5));

            }
            else
            {
                m_motor.Set(m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), 33.0));
            }
        },
        [this]
        {
            m_engaged = !m_engaged;
            m_motor.Set(0.0);
        }
    ).Until([this]{ return m_pid_controller.AtSetpoint(); });
}