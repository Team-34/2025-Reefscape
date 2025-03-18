#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_flipped_out(false)
    , m_lock_flipped_up(true)
    , m_pid_controller(0.2, 0.0, 0.0)
    , m_lock(0)
    {
        m_pid_controller.SetTolerance(0.15);
    }

    



    frc2::CommandPtr Climber::Climb() 
    {
        double setpoint = m_flipped_out ? 0.0 : 120.0;

        m_pid_controller.SetSetpoint(setpoint);

        return this->RunEnd(
        [this]
        {
            m_motor.Set(m_pid_controller.Calculate(m_motor.GetPosition().GetValueAsDouble()));

        }, [this]
        {
            m_motor.Set(0.0);
            FlipLock();
        }).Until(
            [this]
        {
            return m_pid_controller.AtSetpoint();
        });
    }



    frc2::CommandPtr Climber::RunArm(double power)
    {
        return this->RunEnd(
        [this, power]
        {
            m_motor.Set(power);
        },
        [this]
        {
            m_motor.Set(0.0);
        });
    }

    frc2::CommandPtr Climber::RunLock(double power)
    {
        return this->Run(
        [this, power]
        {
            m_lock.Set(power);
        });
    }

    void Climber::FlipLock()
    {
        m_lock.Set((m_lock_flipped_up) ? 0.65 : 0.25);
        m_lock_flipped_up = !m_lock_flipped_up;
    }
}