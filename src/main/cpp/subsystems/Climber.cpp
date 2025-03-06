#include "subsystems/Climber.h"

Climber::Climber() 
: m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_engaged(false)
, m_pid_controller(0.1, 0.0, 0.0)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

frc2::CommandPtr Climber::FlipArmCommand()
{
    static const double engagement_setpoint    { 57.0 };
    static const double disengagement_setpoint { 0.0  };

    return this->RunEnd(
        [this] {
            const auto speed = m_pid_controller.Calculate(
                m_motor.GetEncoder().GetPosition(),
                m_engaged ? engagement_setpoint : disengagement_setpoint);
            m_motor.Set(speed);
        },
        [this]
        {
            m_engaged = !m_engaged;
            m_motor.StopMotor();
        }
    ).Until([this]{ return m_pid_controller.AtSetpoint(); });
}