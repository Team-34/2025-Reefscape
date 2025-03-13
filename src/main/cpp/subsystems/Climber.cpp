#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(5)
    , m_engaged(false)
    , m_pid_controller(0.1, 0.0, 0.0)
    {
        frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    }

    frc2::CommandPtr Climber::FlipArmCommand()
    {
        static const double engagement_speed   {  0.5 };
        static const double disengagement_speed{ 33.0 };

        return this->RunEnd(
            [this] {
                const auto speed = m_pid_controller.Calculate(
                    m_motor.GetPosition().GetValueAsDouble(),
                    m_engaged ? engagement_speed : disengagement_speed);
                m_motor.Set(speed);
            },
            [this]
            {
                m_engaged = !m_engaged;
                m_motor.StopMotor();
            }
        ).Until([this]{ return m_pid_controller.AtSetpoint(); });
    }    
}
