// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveAlgaeWristToCommand.h"

MoveAlgaeWristToCommand::MoveAlgaeWristToCommand(t34::AlgaeIntake *intake, units::degree_t angle) 
: m_intake(intake)
, m_angle(angle)
{
  AddRequirements(intake);
}

void MoveAlgaeWristToCommand::Initialize() {}

void MoveAlgaeWristToCommand::Execute() 
{
  m_intake->MoveAlgaeWristTo(m_angle);
}

void MoveAlgaeWristToCommand::End(bool interrupted) 
{
  m_intake->StopWrist();
}

bool MoveAlgaeWristToCommand::IsFinished() 
{
  return m_intake->EndCondition();
}
