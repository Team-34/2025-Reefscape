#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_engaged(false)
    , m_lock_flipped_up(true)
    , m_pid_controller(0.1, 0.0, 0.0)
    , m_lock(0)
    {
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

                if (m_engaged)
                {
                    m_lock.Set(0.0);
                }
                else
                {
                    m_lock.Set(0.65);
                }
            }
        ).Until([this]{ return m_pid_controller.AtSetpoint(); });
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

    frc2::CommandPtr Climber::FlipLock()
    {
        double go_to = (m_lock_flipped_up) ? 0.4 : 0.0;

        return this->RunEnd(
        [this, go_to]
        {

            m_lock.Set(go_to);
        },

        [this]
        {
            m_lock_flipped_up = !m_lock_flipped_up;
        }
        );
        // ).Until(
        //     [this, go_to] {

        //         return m_lock.GetPosition() == go_to;
        //     });
    }
}