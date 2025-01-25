// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlgaeAutoCommand.h"

AlgaeAutoCommand::AlgaeAutoCommand(t34::SwerveDrive * swerve, Elevator * elevator, t34::Intake * intake) {

  AddRequirements(swerve);

  AddCommands(
    AutoDriveCommand(swerve, units::foot_t(0.0), units::foot_t(0.0), units::degree_t(0.0)), //move to processor
    *intake->RunOut(0.5).Unwrap(), //spit out initial algae
    AutoDriveCommand(swerve, units::foot_t(0.0), units::foot_t(0.0), units::degree_t(0.0)), //move to southwest reef algae

    *elevator->MoveUpOnce().Unwrap(), //move elevator up to level 1
    *elevator->MoveUpOnce().Unwrap(), //move elevator up to level 2
    *intake->RunIn(0.3).Unwrap(), //intake reef algae

    *elevator->MoveDownOnce().Unwrap(), //move elevator down to level 1
    *elevator->MoveDownOnce().Unwrap(), //move elevator down to level 0
    AutoDriveCommand(swerve, units::foot_t(0.0), units::foot_t(0.0), units::degree_t(0.0)), //move to processor
    *intake->RunOut(0.5).Unwrap(), //spit out reef algae

    AutoDriveCommand(swerve, units::foot_t(0.0), units::foot_t(0.0), units::degree_t(0.0)), //move to rightmost algae
    *intake->RunIn(0.3).Unwrap(), //intake floor algae
    AutoDriveCommand(swerve, units::foot_t(0.0), units::foot_t(0.0), units::degree_t(0.0)), //move to processor
    *intake->RunOut(0.5).Unwrap() //spit out floor algae
  );
}
