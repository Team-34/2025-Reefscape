// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevateToCommand.h"

ElevateToCommand::ElevateToCommand(t34::Elevator *elevator, units::inch_t height) 
: m_elevator(elevator)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
}

void ElevateToCommand::Initialize() {}

void ElevateToCommand::Execute() 
{
  m_elevator->ElevateTo(m_height);
}

void ElevateToCommand::End(bool interrupted) 
{
  m_elevator->Stop();
}

bool ElevateToCommand::IsFinished() 
{
  return m_elevator->EndCondition();
}
