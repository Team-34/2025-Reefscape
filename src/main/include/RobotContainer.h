#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include "T34CommandXboxController.h"
#include "commands/ControllerDriveCommand.h"
#include "commands/AutoDriveCommand.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/Climber.h"
#include "subsystems/Elevator.h"
#include "Gyro.h"
#include <frc/AnalogEncoder.h>

class RobotContainer
{
public:
  RobotContainer();
  std::shared_ptr<t34::SwerveDrive> swerve_drive;
  std::shared_ptr<t34::T34CommandXboxController> ctrl;
  t34::ControllerDriveCommand m_default_command;
  t34::AlgaeIntake m_algae_intake;
  t34::CoralIntake m_coral_intake;
  t34::Climber m_climber;
  t34::Elevator m_elevator;
  
  AutoDriveCommand m_auto_leave;

  frc2::CommandPtr GetAutonomousCommand();

private:
  void ConfigureBindings();
};
