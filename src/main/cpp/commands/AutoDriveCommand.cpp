// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveCommand.h"

AutoDriveCommand::AutoDriveCommand(t34::SwerveDrive * swerve, units::foot_t x_translation, units::foot_t y_translation, units::degree_t rotation) 
: m_swerve(swerve)
, m_x_translation(x_translation)
, m_y_translation(y_translation)
, m_rotation(rotation)
, m_x_PID(0.5, 0.0, 0.1)
, m_y_PID(0.5, 0.0, 0.1)
, m_x_init_dist(swerve->GetModulePositions()[0].distance.value())
, m_y_init_dist(swerve->GetModulePositions()[0].distance.value())
{
  AddRequirements(swerve);
}

// Called when the command is initially scheduled.
void AutoDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDriveCommand::Execute() 
{
  m_swerve->Drive(frc::Translation2d(
    units::meter_t(m_x_PID.Calculate(m_swerve->GetModulePositions()[0].distance.value(), (m_x_init_dist + m_x_translation.value()) )),
    units::meter_t(m_y_PID.Calculate(m_swerve->GetModulePositions()[0].distance.value(), (m_y_init_dist + m_y_translation.value()) ))), m_rotation.value()
  );
}

// Called once the command ends or is interrupted.
void AutoDriveCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDriveCommand::IsFinished() {
  return false;
}
