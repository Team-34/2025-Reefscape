// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator()
: m_motors(elevator_motor_id)
, m_motors_PID(1.0, 1.0, 1.0)
{
}

// This method will be called once per scheduler run
void Elevator::Periodic() {}
