// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/controller/PIDController.h>

#include "subsystems/SwerveDrive.h"
#include "commands/ControllerDriveCommand.h"
#include "Constants.h"


class AutoDriveCommand
    : public frc2::CommandHelper<frc2::Command, AutoDriveCommand> {
 public:

  /**
   * @param swerve pointer to the swerve drive subsystem
   * @param x_translation displacement destination towards the right
   * @param y_translation displacement destination in the forward direction
   * @param rotation degrees that the robot should rotate. NOT THE WHEELS.
   * @param tolerance extra inches that the robot will coonsider "zero" at the destination
   * @param speed multiplier for the speed
   */
  AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation, units::inch_t tolerance, double speed = 1.0);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  // inline frc::PIDController GetXPID() { return m_x_PID; };
  // inline frc::PIDController GetYPID() { return m_y_PID; };

  inline units::inch_t GetSetpoint() { return m_setpoint; }
  inline units::inch_t GetTolerance() { return m_tolerance; }
  inline units::inch_t GetXTravelled() { return m_current_x; }
  inline units::inch_t GetYTravelled() { return m_current_y; }
  inline units::inch_t GetTravelled() { return m_travelled; }

  private:

  std::shared_ptr<t34::SwerveDrive> m_swerve;

  units::inch_t m_x_translation;
  units::inch_t m_y_translation;
  units::inch_t m_tolerance;
  units::inch_t m_setpoint;

  units::inch_t m_travelled;
  units::inch_t m_current_x;
  units::inch_t m_current_y;

  units::degree_t m_rotation;
  units::degree_t m_wheel_theta;

  //frc::ChassisSpeeds m_speeds;

  // frc::PIDController m_x_PID;
  // frc::PIDController m_y_PID;
  // frc::PIDController m_r_PID;

  double m_x_drive;
  double m_y_drive;
  double m_x_init_dist;
  double m_y_init_dist;
  double m_r_speed;
  double m_kP;
};
