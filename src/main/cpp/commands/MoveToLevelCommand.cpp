// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveToLevelCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

MoveToLevelCommand::MoveToLevelCommand(t34::Elevator *elevator, t34::AlgaeIntake *algae_intake, t34::CoralIntake *coral_intake, int level) 
: m_elevator(elevator)
, m_algae_intake(algae_intake)
, m_coral_intake(coral_intake)
, m_level(level)

{

  m_level = std::clamp(level, 0, static_cast<int>(presets.size()) - 1);
    
  auto [algae_angle, coral_angle, elevator_height] = presets.at(m_level);

  if (m_level == 0)
  {
    AddCommands(
      MoveCoralWristToCommand(m_coral_intake, coral_angle),
      MoveAlgaeWristToCommand(m_algae_intake, algae_angle),
      ElevateToCommand(m_elevator, elevator_height)
    );
  }
  else
  {
    AddCommands(
      ElevateToCommand(m_elevator, elevator_height),
      MoveCoralWristToCommand(m_coral_intake, coral_angle),
      MoveAlgaeWristToCommand(m_algae_intake, algae_angle)
    );
  }


}
