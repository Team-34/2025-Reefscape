// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
        m_elevator->ElevateTo(1.0);
        //m_elevator->ElevateTo(35.75_in);
        break;
    case 1:
        m_elevator->ElevateTo(2.02);
        //m_elevator->ElevateTo(35.75_in);
        break;
    case 2:
        m_elevator->ElevateTo(5.3);
        break;
    case 3:
        m_elevator->ElevateTo(9.9);
        break;
    }

}

frc2::CommandPtr t34::Coordinator::RunElevator(double speed)
{
    return m_coral_intake->MoveToZero().AndThen(m_elevator->MoveElevatorByPowerCommand(speed));
}

// This method will be called once per scheduler run
void t34::Coordinator::Periodic() 
{
    frc::SmartDashboard::PutNumber("Level", m_current_level);
}
