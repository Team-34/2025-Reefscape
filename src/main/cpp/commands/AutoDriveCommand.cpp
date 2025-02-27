// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "commands/AutoDriveCommand.h"

// /**
//  *  @param swerve a shared_ptr to the swerve drive subsystem that you're using
//  *  @param x_translation the change in horizontal distance from the starting 
//  */
// AutoDriveCommand::AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation, units::inch_t tolerance) 
// : m_swerve(swerve)
// , m_x_translation(x_translation)
// , m_y_translation(y_translation)
// , m_tolerance(tolerance)
// , m_setpoint(std::sqrt((m_x_translation.value() * m_x_translation.value()) + (m_y_translation.value() * m_y_translation.value())))
// , m_travelled(0.0)
// , m_current_x(0.0)
// , m_current_y(0.0)
// , m_base_rotation(rotation)
// , m_wheel_theta(std::atan(m_y_translation.value() / m_x_translation.value()))
// , m_current_theta(m_swerve->GetYaw().Degrees())
// , m_x_drive(0.0)
// , m_y_drive(0.0)
// , m_init_dist(swerve->GetModulePositions()[0].distance.value())
// , m_r_speed(0.0)
// , m_theta_speed(0.0)
// {
//   AddRequirements(swerve.get());

// }

// // Called when the command is initially scheduled.
// void AutoDriveCommand::Initialize() {}

// // Called repeatedly when this Command is scheduled to run
// void AutoDriveCommand::Execute() 
// {

//   m_travelled = units::inch_t(m_swerve->GetModulePositions()[0].distance.value() - m_init_dist);
//   m_current_x = units::inch_t(-m_travelled * std::cos(m_wheel_theta.value()));
//   m_current_y = units::inch_t(-m_travelled * std::sin(m_wheel_theta.value()));
//   m_current_theta = m_swerve->GetYaw().Degrees();

//   //m_x_drive = 8 * ((m_x_translation.value() - m_current_x.value()) / (m_x_translation.value() == 0.0 ? 1.0 : m_x_translation.value()));
//   //m_y_drive = -8 * ((m_y_translation.value() - m_current_y.value()) / (m_y_translation.value() == 0.0 ? 1.0 : m_y_translation.value())) * std::tan(m_wheel_theta.value());
//   //m_theta_speed = m_kP * ((m_wheel_theta.value() - m_current_theta.value()) / (m_wheel_theta.value() == 0.0 ? 1.0 : m_wheel_theta.value()));

//   m_x_drive = 8 * ((m_setpoint.value() - m_travelled.value()) / (m_x_translation.value() == 0.0 ? 1.0 : m_setpoint.value()));
//   m_y_drive = -8 * ((m_setpoint.value() - m_travelled.value()) / (m_y_translation.value() == 0.0 ? 1.0 : m_setpoint.value())) * std::tan(m_wheel_theta.value());
//   m_theta_speed = 10 * ((m_wheel_theta.value() - m_current_theta.value()) / (m_wheel_theta.value() == 0.0 ? 1.0 : m_wheel_theta.value()));

//   if ((std::fabs(m_setpoint.value()) - m_tolerance.value()) < m_travelled.value()
//    && m_travelled.value() < (std::fabs(m_setpoint.value()) + m_tolerance.value()))
//   {
//     m_swerve->Stop();
//     m_swerve->ResetToAbsolute();
//   }
//   else
//   {

//     if ((m_x_translation.value() - m_tolerance.value()) < m_current_x.value() 
//     && m_current_x.value() < (m_x_translation.value() + m_tolerance.value()))
//     {
//       m_x_drive = 0.0;
//     }
//     if ((m_y_translation.value() - m_tolerance.value()) < m_current_y.value() 
//      && m_current_y.value() < (m_y_translation.value() + m_tolerance.value()))
//     {
//       m_y_drive = 0.0;
//     }

//     m_swerve->Drive(frc::Translation2d(
//     units::meter_t(m_x_drive),
//     units::meter_t(m_y_drive)),
//     m_theta_speed
//     );
//   }

// }

// // Called once the command ends or is interrupted.
// void AutoDriveCommand::End(bool interrupted) {}

// // Returns true when the command should end.
// bool AutoDriveCommand::IsFinished() {
//   return false;
// }
