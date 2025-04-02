#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_flipped_out(false)
    , m_lock_flipped_up(true)
    , m_lock(0)
    , m_at_locking_position([this] { return AtLockingPosition(); })
    {
        ctre::phoenix6::configs::Slot0Configs slot0Configs;
        slot0Configs.kP = 0.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.001;

        m_motor.GetConfigurator().Apply(slot0Configs);

        m_at_locking_position.OnTrue(LockCommand());
    }

    frc2::CommandPtr Climber::Climb() 
    {
        return this->RunOnce(
        [this]
        {
            auto setpoint = m_flipped_out ? 0.0_tr : 115.4_tr;
            auto request = ctre::phoenix6::controls::PositionVoltage{0_tr};

            m_motor.SetControl(request.WithPosition(setpoint));

            m_flipped_out = !m_flipped_out;
        }).BeforeStarting(UnlockCommand());
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

    frc2::CommandPtr Climber::LockCommand()
    {
        return this->RunOnce(
            [this]
            {
                Lock();  
            }
        );
    }

    frc2::CommandPtr Climber::UnlockCommand()
    {
        return this->RunOnce( [this] { Unlock(); } ).AndThen(frc2::cmd::Wait(1_s));
    }

    void Climber::Periodic()
    {
        frc::SmartDashboard::PutNumber("Climber Encoder Val: ", m_motor.GetPosition().GetValueAsDouble());
        frc::SmartDashboard::PutBoolean("Climber Flipped Out ?", m_flipped_out);
        frc::SmartDashboard::PutBoolean("At Locking Position ?", AtLockingPosition());
       // SetLock();
    }
}