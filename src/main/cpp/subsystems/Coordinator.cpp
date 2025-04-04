// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Coordinator.h"

t34::Coordinator::Coordinator(t34::Elevator* elevator)
: m_elevator(elevator)
, m_current_level(0)
{}

frc2::CommandPtr t34::Coordinator::MoveToLevelCommand(int level)
{
    m_current_level = std::clamp(level, 0, 3);

    switch (m_current_level)
    {
        case 0:
            return m_elevator->ElevateToCommand(8.9);
        case 1:
            return m_elevator->ElevateToCommand(9.56);
        case 2:
            return m_elevator->ElevateToCommand(1.49);
        case 3:
            return m_elevator->ElevateToCommand(0.0);
    }

}

// This method will be called once per scheduler run
void t34::Coordinator::Periodic() {}
