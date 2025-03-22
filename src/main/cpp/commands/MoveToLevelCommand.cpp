// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveToLevelCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

MoveToLevelCommand::MoveToLevelCommand(t34::CoralIntake *coral_intake, t34::AlgaeIntake *algae_intake, t34::Elevator *elevator, int level) 
: m_coral_intake(coral_intake)
, m_algae_intake(algae_intake)
, m_elevator(elevator)
, m_level(level)

{

  m_level = std::clamp(level, 0, static_cast<int>(presets.size()) - 1);
    
  auto [algae_angle, coral_angle, elevator_height] = presets.at(level);

  if (m_level == 0)
  {
    AddCommands( 
      *m_algae_intake->MoveAlgaeWristToCommand(algae_angle).Unwrap().get(), 
      *m_coral_intake->MoveCoralWristToCommand(coral_angle).Unwrap().get(),
      *m_elevator->ElevateToCommand(elevator_height).Unwrap().get()
    );
  }
  else
  {
    AddCommands(
      *m_elevator->ElevateToCommand(elevator_height).Unwrap().get(), 
      *m_algae_intake->MoveAlgaeWristToCommand(algae_angle).Unwrap().get(), 
      *m_coral_intake->MoveCoralWristToCommand(coral_angle).Unwrap().get()
    );
  }


}
