// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoScoreAlgaeAndCoralCommand.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutoScoreAlgaeAndCoralCommand::AutoScoreAlgaeAndCoralCommand(std::shared_ptr<t34::SwerveDrive> swerve) 
: m_swerve(swerve)
{

AddRequirements(m_swerve.get());

  AddCommands(
    AutoDriveCommand(m_swerve, 5.123_ft, -8.108_ft, 90_deg), //from starting point to processor
    AutoDriveCommand(m_swerve, -11.174_ft, 20.896_ft, -45_deg), //from processor to reef
    CenterOnATCommand(m_swerve),
    AutoDriveCommand(m_swerve, 11.174_ft, -20.896_ft, 90_deg), //from reef to processor
    AutoDriveCommand(m_swerve, -5.123_ft, 13.058_ft, 0_deg) //from processor to 4.95ft away from the starting position
    
  );
}
