// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CenterOnCoral.h"

CenterOnCoral::CenterOnCoral(t34::SwerveDrive * swerve_ptr)
: m_LL("T34 LL", units::degree_t(0))
, m_swerve(swerve_ptr)
, m_x_PID(0.1, 0.0, 0.1)
, m_y_PID(0.1, 0.0, 0.1)
{
  m_x_PID.SetTolerance(0.2);
  m_y_PID.SetTolerance(0.15);

  AddRequirements(m_swerve);
}

// Called when the command is initially scheduled.
void CenterOnCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CenterOnCoral::Execute()
{

  m_swerve->Drive(frc::Translation2d(units::meter_t(

    //(-0.2 < m_LL.GetNearestAT().txnc < 0.2) ? 0.0 : std::copysign(0.05, m_LL.GetNearestAT().txnc)
    m_x_PID.Calculate(m_LL.GetNearestAT().txnc, 0.0)

  ), units::meter_t(

    //(m_LL.GetNearestAT().distToCamera > 0.1) ? 0.05 : 0.0
    m_y_PID.Calculate(m_LL.GetNearestAT().distToCamera, 0.1)

  )), 0.0);

}

// Called once the command ends or is interrupted.
void CenterOnCoral::End(bool interrupted) {}

// Returns true when the command should end.
bool CenterOnCoral::IsFinished() {
  return false;
}
