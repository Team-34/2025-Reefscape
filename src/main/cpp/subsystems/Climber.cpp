#include "subsystems/Climber.h"

t34::Climber::Climber() 
: m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_engaged(false)
, m_pid_controller(0.1, 0.0, 0.0)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

frc2::CommandPtr t34::Climber::FlipArmCommand()
{
    double setpoint = Neo::AngleToNEOUnit(m_engaged ? 0.5_deg : 37_deg);

    return this->RunEnd(

        [this, setpoint] 
        {
            m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), setpoint);
        },
        [this]
        {
            m_engaged = !m_engaged;
            m_motor.StopMotor();
        }

    ).Until([this]{ return m_pid_controller.AtSetpoint(); });
}