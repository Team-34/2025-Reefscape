// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/ElevateToCommand.h"
#include "commands/MoveAlgaeWristToCommand.h"
#include "commands/MoveCoralWristToCommand.h"

#include "subsystems/Elevator.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/CoralIntake.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

class MoveToLevelCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 MoveToLevelCommand> {
  public:
  MoveToLevelCommand(t34::Elevator *elevator, t34::AlgaeIntake *algae_intake, t34::CoralIntake *coral_intake, int level);

  private:

  t34::Elevator *m_elevator;
  t34::AlgaeIntake *m_algae_intake;
  t34::CoralIntake *m_coral_intake;

  int m_level;

                            //Algae Intake    Coral Intake   Elevator height
const std::array<std::tuple<units::degree_t, units::degree_t, units::inch_t>, 5> presets {{
  { 0_deg, 5_deg, 3_in }, //The swerve modules are 2.75 inches above the elevator's lowest point. The 775Pros will collide with the modules if they are ran all the way to the bottom.
  { 90_deg, 55_deg, 18_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
  { 90_deg, 55_deg, 31.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
  { 90_deg, 55_deg, 47.875_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
  { 90_deg, 2_deg, 72_in - BASE_HEIGHT_FROM_FLOOR - ELEVATOR_LOWEST_POINT_FROM_BASE },
}};

};
