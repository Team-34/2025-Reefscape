#include "subsystems/Coordinator.h"

t34::Coordinator::Coordinator(t34::Elevator* elevator, t34::CoralIntake* coral_intake)
: m_elevator(elevator)
, m_coral_intake(coral_intake)
, m_current_level(0)
{
    
}

void t34::Coordinator::MoveToLevel(int level)
{
    frc::SmartDashboard::PutNumber("Level moving to", level);

    m_current_level = std::clamp(level, 0, 3);

    switch (m_current_level)
    {
    case 0:
        m_elevator->ElevateTo(0.0);
        break;
    case 1:
        m_elevator->ElevateTo(1.586);
        break;
    case 2:
        m_elevator->ElevateTo(5.3);
        break;
    case 3:
        m_elevator->ElevateTo(9.99);
        m_coral_intake->MoveWristTo(9.357);
        break;
    }

}

frc2::CommandPtr t34::Coordinator::RunElevator(double speed)
{
    return m_coral_intake->MoveToZero().AndThen(m_elevator->MoveElevatorByPowerCommand(speed));
}

void t34::Coordinator::Periodic() 
{
    frc::SmartDashboard::PutNumber("Level", m_current_level);
}
