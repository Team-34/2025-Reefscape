// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreAlgae2Coral.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ScoreAlgae2Coral::ScoreAlgae2Coral(std::shared_ptr<t34::SwerveDrive> swerve) 
: m_swerve(swerve)
{
  // Add your commands here, e.g.

  AddRequirements(m_swerve.get());

  AddCommands
  (

    AutoDriveCommand(m_swerve, 13.635_ft, 19.606_ft, 90_deg),
    CenterOnATCommand(m_swerve),
    AutoDriveCommand(m_swerve, 5.333_ft, 0_ft, 0_deg),
    AutoDriveCommand(m_swerve, 6.051_ft, 14.563_ft, 45_deg),
    CenterOnATCommand(m_swerve),
    AutoDriveCommand(m_swerve, 8.431_ft, 10.323_ft, -135_deg),
    CenterOnATCommand(m_swerve)
   
  );
}
