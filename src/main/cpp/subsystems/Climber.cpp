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
    //static const double engagement_speed   {  0.5 };
    //static const double disengagement_speed{ 33.0 };

    double setpoint = (m_engaged == false ) ? Neo::AngleToNEOUnit(37_deg) : Neo::AngleToNEOUnit(0.5_deg);

    return this->RunEnd(

        [this, setpoint] {
            // const auto speed = m_pid_controller.Calculate(
            //     m_motor.GetEncoder().GetPosition(),
            //     m_engaged ? engagement_speed : disengagement_speed);
            // m_motor.Set(speed);

            m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), setpoint);

        },
        [this]
        {
            m_engaged = !m_engaged;
            m_motor.StopMotor();
        }

    ).Until([this]{ return m_pid_controller.AtSetpoint(); });
}