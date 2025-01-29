#include "subsystems/Climber.h"

Climber::Climber() 
: m_pid_controller(0.1, 0.0, 0.0)
, m_motor(5, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_climber_up(false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}

// frc2::CommandPtr Climber::MoveInc() {
//     return this->StartEnd(
//         [this] {
//             m_motor.Set(0.1);
//         },
//         [this] {
//             m_motor.Set(0.0);
//         }

//     );
// }

frc2::CommandPtr Climber::FlipArm()
{
    return this->RunEnd(
        [this] {
            if (m_climber_up){
                m_motor.Set(m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), 0.5));//NEOUnitToDegree(37.0));

            }
            else
            {
                m_motor.Set(m_pid_controller.Calculate(m_motor.GetEncoder().GetPosition(), 30.0));
            }
        },
        [this]
        {
            m_climber_up = !m_climber_up;
            m_motor.Set(0.0);
        }
    ).Until([this]{ return m_pid_controller.AtSetpoint(); });
}
