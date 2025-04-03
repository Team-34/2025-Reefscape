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
    this->RunOnce(
        [this, level]
        {
            m_current_level = std::clamp(level, 0, 4);
        }
    ).AndThen(m_elevator->ElevateToCommand(level));
}

// This method will be called once per scheduler run
void t34::Coordinator::Periodic() {}
