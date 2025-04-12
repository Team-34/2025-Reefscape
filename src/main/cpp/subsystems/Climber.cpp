#include "subsystems/Climber.h"

namespace t34
{
    Climber::Climber()
    : m_motor(37)
    , m_deployed(false)
    , m_locked(true)
    , m_slot_0_configs()
    , m_lock(0)
    {
        m_slot_0_configs.kP = 0.3;
        m_slot_0_configs.kI = 0;
        m_slot_0_configs.kD = 0.0005;

        ctre::phoenix6::configs::ClosedLoopRampsConfigs ramp_config;

        ramp_config.WithVoltageClosedLoopRampPeriod(0.5_s);

        m_motor.GetConfigurator().Apply(m_slot_0_configs);
        m_motor.GetConfigurator().Apply(ramp_config);
    }

    frc2::CommandPtr Climber::Climb() 
    {

        return this->RunOnce(
            [this]
            {
                m_motor.StopMotor();
            }
        )
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(ToggleLockCommand())
        .AndThen(ToggleKPCommand())
        .AndThen(frc2::cmd::Wait(1_s))
        .AndThen(ToggleDeploymentCommand());
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

    frc2::CommandPtr Climber::ToggleLockCommand()
    {

        return frc2::cmd::RunOnce(
            [this]
            {
                m_locked ? Lock() : Unlock();
                m_locked = !m_locked;
            }
        );
    }

    void Climber::Deploy()
    {
        m_motor.SetControl(m_request.WithPosition(115_tr));
    }

    void Climber::Retract()
    {
        m_motor.SetControl(m_request.WithPosition(0_tr));
    }

    frc2::CommandPtr Climber::ToggleDeploymentCommand()
    {
        this->RunOnce(
            [this]
            {
                m_deployed ? Retract() : Deploy();

                m_deployed = !m_deployed;
            }
        );
    }

    frc2::CommandPtr Climber::ToggleKPCommand()
    {
        m_slot_0_configs.kP = m_deployed ? 0.15 : 0.3;
        m_motor.GetConfigurator().Apply(m_slot_0_configs);
    }

    void Climber::Periodic()
    {
        frc::SmartDashboard::PutNumber("Climber Encoder Val: ", m_motor.GetPosition().GetValueAsDouble());
        frc::SmartDashboard::PutBoolean("Climber Locked?", m_locked);
    }
}