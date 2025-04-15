#include "subsystems/Coordinator.h"

t34::Coordinator::Coordinator(t34::Elevator* elevator, t34::CoralIntake* coral_intake)
: m_elevator(elevator)
, m_coral_intake(coral_intake)
, m_current_level(0)
{
    
}

frc2::CommandPtr t34::Coordinator::MoveToLevelCommand(int level)
{
    frc2::CommandPtr cmd = frc2::cmd::None();

    m_current_level = std::clamp(level, 0, 3);

    frc::SmartDashboard::PutNumber("Level moving to: ", m_current_level);

    double elev_pos;
    double co_pos;

    return this->RunOnce(
    [this, level, &cmd, &elev_pos, &co_pos]
    {
    switch (m_current_level)
    {
    case 0:
        // cmd = m_coral_intake->MoveToZero()
        // .AndThen(m_elevator->ElevateToCommand(0.0))
        // .AndThen(m_coral_intake->MoveWristToCommand(12.0));
        elev_pos = 0.0;
        co_pos = 12.0;
        break;
    case 1:
        // cmd = m_coral_intake->MoveToZero()
        // .AndThen(m_elevator->ElevateToCommand(1.586))
        // .AndThen(m_coral_intake->MoveWristToCommand(12.0));
        elev_pos = 1.586;
        co_pos = 12.0;
        break;
    case 2:
        // cmd = m_coral_intake->MoveToZero()
        // .AndThen(m_elevator->ElevateToCommand(5.3))
        // .AndThen(m_coral_intake->MoveWristToCommand(12.0));
        elev_pos = 5.3;
        co_pos = 12.0;
        break;
    case 3:
        // cmd = m_coral_intake->MoveWristToCommand(9.357)
        // .AndThen(m_elevator->ElevateToCommand(9.99))
        // .AndThen(m_coral_intake->MoveWristToCommand(12.0));
        elev_pos = 9.9;
        co_pos = 9.357;
        break;
    }
    }
    ).AndThen(m_coral_intake->MoveToZero()
            .AndThen(m_elevator->ElevateToCommand(elev_pos))
            .AndThen(m_coral_intake->MoveWristToCommand(co_pos)))
    .AndThen(
        this->RunOnce
        (
            [this, &elev_pos, &co_pos] 
                {
                    elev_pos = 0.0;
                    co_pos = 0.0;
                }
        )
        
    );
  
}

void t34::Coordinator::MoveToLevel(int level)
{
    m_current_level = std::clamp(level, 0, 3);

    frc::SmartDashboard::PutNumber("Level moving to: ", m_current_level);
        
    switch (m_current_level)
    {
    case 0:
        m_elevator->ElevateToCommand(0.0);
        m_coral_intake->MoveWristToCommand(12.0);
        break;
    case 1:
        m_elevator->ElevateToCommand(1.586);
        m_coral_intake->MoveWristToCommand(12.0);
        break;
    case 2:
        m_elevator->ElevateToCommand(5.3);
        m_coral_intake->MoveWristToCommand(12.0);
        break;
    case 3:
        m_elevator->ElevateToCommand(9.99);
        m_coral_intake->MoveWristToCommand(9.357);
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
