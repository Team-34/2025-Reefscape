// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveCommand.h"

using namespace units::math;

/**
 *  @param swerve a shared_ptr to the swerve drive subsystem that you're using
 *  @param x_translation the change in horizontal distance from the starting 
 */
AutoDriveCommand::AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation) 
: m_swerve(swerve)
, m_x_translation(x_translation)
, m_y_translation(y_translation)
, m_setpoint(sqrt((m_x_translation * m_x_translation) + (m_y_translation * m_y_translation)))
, m_travelled(0.0)
, m_current_x(0.0)
, m_current_y(0.0)
, m_base_rotation(rotation)
, m_wheel_theta(atan(m_y_translation / m_x_translation))
, m_current_theta(m_swerve->GetYaw().Degrees())
, m_x_drive(0.0)
, m_y_drive(0.0)
, m_drive_pid(1.0, 0.0, 0.045)
, m_rot_pid(0.5, 0.0, 0.1)
, m_init_dist(swerve->GetModulePositions()[0].distance.value())
, m_rot_speed(0.0)
, m_invert_drives( (m_x_translation.value() >= 0.0) ? 1.0 : -1.0 )
, m_at_drive_setpoint(false)
{
  AddRequirements(swerve.get());
}

// Called when the command is initially scheduled.
void AutoDriveCommand::Initialize() 
{
  m_drive_pid.SetSetpoint(m_setpoint.value());
  m_drive_pid.SetTolerance(1.0);
  m_rot_pid.SetSetpoint(m_base_rotation.value());
  m_rot_pid.SetTolerance(1.0);
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveCommand::Execute() 
{
  m_travelled = EncUnitsToInches(m_swerve->GetModulePositions()[0].distance.value() + m_init_dist);
  m_current_x = m_travelled * cos(m_wheel_theta);
  m_current_y = -m_travelled * sin(m_wheel_theta);
  m_current_theta = m_swerve->GetYaw().Degrees();
  m_y_drive = m_drive_pid.Calculate(m_travelled.value());
  m_x_drive = m_y_drive * tan(m_wheel_theta);
  m_rot_speed = m_rot_pid.Calculate(m_current_theta.value());
  
    if (fabs(m_travelled) < fabs(m_setpoint) - 1_in)
    {
      m_swerve->Drive(frc::Translation2d(
      units::meter_t(m_invert_drives * m_y_drive),
      units::meter_t(-m_x_drive)),
      m_rot_speed
    );

    m_at_drive_setpoint = false;
    }
    else
    {
      m_swerve->Drive(frc::Translation2d(0_m, 0_m), m_rot_speed);
      m_at_drive_setpoint = true;
    }
  }

// Called once the command ends or is interrupted.
void AutoDriveCommand::End(bool interrupted) {
  m_swerve->ResetToAbsolute();
  m_swerve->Stop();
}

// Returns true when the command should end.
bool AutoDriveCommand::IsFinished() {
  return m_at_drive_setpoint;
}