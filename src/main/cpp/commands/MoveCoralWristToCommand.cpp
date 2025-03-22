// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveCoralWristToCommand.h"

MoveCoralWristToCommand::MoveCoralWristToCommand(t34::CoralIntake *intake, units::degree_t angle) 
: m_intake(intake)
, m_encoder_setpoint(t34::Neo::AngleTo550Unit(angle))
{}

// Called when the command is initially scheduled.
void MoveCoralWristToCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveCoralWristToCommand::Execute() 
{
  m_intake->MoveCoralWristTo(m_encoder_setpoint);
}

// Called once the command ends or is interrupted.
void MoveCoralWristToCommand::End(bool interrupted) 
{
  m_intake->StopWrist();
}

// Returns true when the command should end.
bool MoveCoralWristToCommand::IsFinished()
{
  return m_intake->EndCondition();
}
