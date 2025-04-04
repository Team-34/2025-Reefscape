#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_deployed(false)
    , m_locked(true)
    , m_lock(0)
    {
        ctre::phoenix6::configs::Slot0Configs slot0Configs;
        slot0Configs.kP = 0.3;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.0005;

        m_motor.GetConfigurator().Apply(slot0Configs);
    }

    frc2::CommandPtr Climber::Climb() 
    {
        //auto setpoint = m_deployed ? 0_tr : 100_tr;
        //auto request = ctre::phoenix6::controls::PositionVoltage{0_tr};

        //m_motor.SetControl(m_request.WithPosition(setpoint));

        return this->RunOnce(
            [this]
            {
                m_motor.StopMotor();
            }
        )
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(
            [this]
            {
                m_locked ? Unlock() : Lock();

                m_locked = !m_locked;
            }
        )
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(
            [this]
            {
                m_deployed ? Retract() : Deploy();

                m_deployed = !m_deployed;
            }
        );
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

    frc2::CommandPtr Climber::FlipLockCommand()
    {

        return frc2::cmd::RunOnce(
            [this]
            {
                m_locked = !m_locked;

                m_locked ? Lock() : Unlock();
            }
        );
    }

    void Climber::Deploy()
    {
        m_motor.SetControl(m_request.WithPosition(110_tr));
    }

    void Climber::Retract()
    {
        m_motor.SetControl(m_request.WithPosition(0_tr));
    }

    void Climber::Periodic()
    {
        frc::SmartDashboard::PutNumber("Climber Encoder Val: ", m_motor.GetPosition().GetValueAsDouble());
        frc::SmartDashboard::PutBoolean("Climber Locked?", m_locked);
    }
}