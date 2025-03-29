#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>
#include <units/angle.h>
#include <cmath>
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
   * @param tolerance extra inches that the robot will consider "zero" at the destination
   */
  AutoDriveCommand(std::shared_ptr<t34::SwerveDrive> swerve, units::inch_t x_translation, units::inch_t y_translation, units::degree_t rotation);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  // inline frc::PIDController GetXPID() { return m_x_PID; };
  // inline frc::PIDController GetYPID() { return m_y_PID; };

  inline units::inch_t GetSetpoint() { return m_setpoint; }
  inline units::inch_t GetXTravelled() { return m_current_x; }
  inline units::inch_t GetYTravelled() { return m_current_y; }
  inline units::inch_t GetTravelled() { return m_travelled; }
  
  inline double GetXOutput() { return m_x_drive; }
  inline double GetYOutput() { return m_y_drive; }
  inline double GetSteerOutput() { return m_rot_speed; }

  private:

  inline units::inch_t EncUnitsToInches(double units) { return units::inch_t(units * (77.5/86)); } // 77.5 inches / 86 encoder units
  inline units::inch_t EncUnitsToInches(units::inch_t inches) { return units::inch_t(inches.value() * (77.5/86)); }

  std::shared_ptr<t34::SwerveDrive> m_swerve;

  units::inch_t m_x_translation;
  units::inch_t m_y_translation;
  units::inch_t m_setpoint;

  units::inch_t m_travelled;
  units::inch_t m_current_x;
  units::inch_t m_current_y;

  units::degree_t m_base_rotation;
  units::degree_t m_wheel_theta;
  units::degree_t m_current_theta;

  frc::PIDController m_drive_pid;
  frc::PIDController m_rot_pid;

  double m_x_drive;
  double m_y_drive;
  double m_rot_speed;
  
  double m_init_dist;
  double m_invert_drives;

  bool m_at_drive_setpoint;
};
