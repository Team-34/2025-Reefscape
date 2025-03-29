#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_flipped_out(false)
    , m_lock_flipped_up(true)
    , m_lock(0)
    {
        ctre::phoenix6::configs::Slot0Configs slot0Configs;
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.001;

        m_motor.GetConfigurator().Apply(slot0Configs);
    }

    frc2::CommandPtr Climber::Climb() 
    {
        units::turn_t setpoint = m_flipped_out ? 0.0_tr : 33.0_tr;

        return this->RunOnce(
        [this, setpoint]
        {
            auto request = ctre::phoenix6::controls::PositionVoltage{0_tr};
            m_motor.SetControl(request.WithPosition(setpoint));
        });
    }

    frc2::CommandPtr Climber::RunArmBySpeed(double speed)
    {
        return this->RunEnd(
        [this, speed]
        {
            m_motor.Set(speed);
        },
        [this]
        {
            m_motor.Set(0.0);
        });
    }

    frc2::CommandPtr Climber::RunLock(double position)
    {
        return this->RunOnce(
        [this, position]
        {
            m_lock.Set(position);
        });
    }

    void Climber::FlipLock()
    {
        m_lock.Set((m_lock_flipped_up) ? 0.65 : 0.25);
        m_lock_flipped_up = !m_lock_flipped_up;
    }
}