// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveCommand.h"

using namespace units::math;

/**
 *  @param swerve a shared_ptr to the swerve drive subsystem that you're using
 *  @param x_translation the change in horizontal distance from the starting 
 */
AutoDriveCommand::AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation, units::inch_t tolerance) 
: m_swerve(swerve)
, m_x_translation(x_translation)
, m_y_translation(y_translation)
, m_tolerance(tolerance)
, m_setpoint(sqrt((m_x_translation * m_x_translation) + (m_y_translation * m_y_translation)))
, m_travelled(0.0)
, m_current_x(0.0)
, m_current_y(0.0)
, m_base_rotation(rotation)
, m_wheel_theta(atan(m_y_translation / m_x_translation))
, m_current_theta(m_swerve->GetYaw().Degrees())
, m_x_drive(0.0)
, m_y_drive(0.0)
, m_init_dist(swerve->GetModulePositions()[0].distance.value())
, m_theta_speed(0.0)
, m_drive_tolerance(m_tolerance / m_setpoint)
{
  AddRequirements(swerve.get());

}

// Called when the command is initially scheduled.
void AutoDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoDriveCommand::Execute() 
{

  m_travelled = units::inch_t(-m_swerve->GetModulePositions()[0].distance.value() + m_init_dist);
  m_current_x = m_travelled * cos(m_wheel_theta);
  m_current_y = -m_travelled * sin(m_wheel_theta);
  m_current_theta = m_swerve->GetYaw().Degrees();
  
  m_x_drive = -8_in * ((m_x_translation - m_current_x) / (m_x_translation == 0_in ? 1_in : m_x_translation));
  m_y_drive = -8_in * ((m_y_translation - m_current_y) / (m_y_translation == 0_in ? 1_in : m_y_translation)) * tan(m_wheel_theta);

  // m_x_drive = 4_in * ((m_setpoint - m_travelled) / (m_setpoint == 0_in ? 1_in : m_setpoint));
  // m_y_drive = m_x_drive * tan(m_wheel_theta);

  m_theta_speed = 1_deg * ((m_wheel_theta - m_current_theta) / (m_wheel_theta == 0_deg ? 1_deg : m_wheel_theta));

  if ((abs(m_setpoint) - m_tolerance) < m_travelled
   && m_travelled < (abs(m_setpoint) + m_tolerance))
  {
    m_swerve->Stop();
    m_swerve->ResetToAbsolute();
  }
  else
  {

    // if ((m_x_translation - m_tolerance) < m_current_x 
    //   && m_current_x < (m_x_translation + m_tolerance))
    // {
    //   m_x_drive = 0_in;
    // }
    // if ((m_y_translation - m_tolerance) < m_current_y 
    //   && m_current_y < (m_y_translation + m_tolerance))
    // {
    //   m_y_drive = 0_in;
    // }

    if (-m_drive_tolerance < units::scalar_t(m_x_drive.value()) && units::scalar_t(m_x_drive.value()) < m_drive_tolerance)
    {
      m_x_drive = 0_in;
    }
    if (-m_drive_tolerance < units::scalar_t(m_y_drive.value()) && units::scalar_t(m_y_drive.value()) < m_drive_tolerance)
    {
      m_y_drive = 0_in;
    }

    m_swerve->Drive(frc::Translation2d(
    units::meter_t(m_x_drive.value()),
    units::meter_t(m_y_drive.value())),
    m_theta_speed.value()
    );
  }

}

// Called once the command ends or is interrupted.
void AutoDriveCommand::End(bool interrupted) {
  m_swerve->ResetToAbsolute();
}

// Returns true when the command should end.
bool AutoDriveCommand::IsFinished() {
  return false;
}
