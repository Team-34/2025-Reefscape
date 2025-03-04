// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CenterOnATCommand.h"

CenterOnATCommand::CenterOnATCommand(std::shared_ptr<t34::SwerveDrive> swerve_ptr)
: m_LL("", units::degree_t(0))
, m_swerve(swerve_ptr)
, m_x_PID(0.1, 0.0, 0.1)
, m_y_PID(0.1, 0.0, 0.1)
{
  m_x_PID.SetTolerance(0.1);
  m_y_PID.SetTolerance(0.1);

  AddRequirements(m_swerve.get());
}

// Called when the command is initially scheduled.
void CenterOnATCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CenterOnATCommand::Execute()
{

  m_swerve->Drive(frc::Translation2d(units::meter_t(
    m_x_PID.Calculate(m_LL.GetTX(), 0.0)
  ), units::meter_t(
    m_y_PID.Calculate(m_LL.CalcDistance().value(), 6)
  )), 0.0);

}

// Called once the command ends or is interrupted.
void CenterOnATCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool CenterOnATCommand::IsFinished() {
  return (m_x_PID.AtSetpoint() && m_y_PID.AtSetpoint());
}
