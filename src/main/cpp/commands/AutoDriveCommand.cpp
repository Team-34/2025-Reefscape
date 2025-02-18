// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveCommand.h"

AutoDriveCommand::AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation) 
: m_swerve(swerve)
, m_x_translation(x_translation)
, m_y_translation(y_translation)
, m_rotation(rotation)
, m_speeds()
, m_x_PID(0.5, 0.0, 0.1)
, m_y_PID(0.5, 0.0, 0.1)
, m_r_PID(0.5, 0.0, 0.1)
, m_x_drive(0.0)
, m_y_drive(0.0)
, m_x_init_dist(swerve->GetModulePositions()[0].distance.value())
, m_y_init_dist(swerve->GetModulePositions()[0].distance.value())
, m_r_speed(0.0)

{
  AddRequirements(swerve.get());
  m_x_PID.SetTolerance(5.0);
  m_y_PID.SetTolerance(5.0);
  m_r_PID.SetTolerance(5.0);
}

// Called when the command is initially scheduled.
void AutoDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDriveCommand::Execute() 
{
  m_x_drive = m_x_PID.Calculate(m_swerve->GetModulePositions()[0].distance.value(), (m_x_init_dist + m_x_translation.value()) );
  m_y_drive = m_y_PID.Calculate(m_swerve->GetModulePositions()[0].distance.value(), (m_y_init_dist + m_y_translation.value()) );
  m_r_speed = m_r_PID.Calculate(m_swerve->GetYaw().Degrees().value(), (m_rotation.value()) );

  if (m_x_PID.AtSetpoint())
  {
    m_x_drive = 0.0;
  }

  if (m_y_PID.AtSetpoint())
  {
    m_y_drive = 0.0;
  }

  if (m_r_PID.AtSetpoint())
  {
    m_r_speed = 0.0;
  }

  m_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    FPSToMPS(m_x_drive * MAX_DRIVE_SPEED_AUTO * -1.0),
    FPSToMPS(m_y_drive * MAX_DRIVE_SPEED_AUTO * -1.0),
    units::radians_per_second_t(DEG_TO_RAD(m_r_speed)),
    frc::Rotation2d(m_swerve->GetYaw().Degrees() * -1.0)
  );

  auto sms = m_swerve->GetKinematics().ToSwerveModuleStates(m_speeds);

  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&sms, units::feet_per_second_t(MAX_DRIVE_SPEED_AUTO));

  for(size_t i = 0; i < m_swerve->GetModules().size(); ++i) {
            m_swerve->GetModules()[i].SetDesiredState(sms[i], false);
  }
}

// Called once the command ends or is interrupted.
void AutoDriveCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoDriveCommand::IsFinished() {
  return false;
}
