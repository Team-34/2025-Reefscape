// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

static std::unique_ptr<RobotContainer> g_rc{ nullptr };


RobotContainer* RobotContainer::Get() {
    if (!g_rc) {
        g_rc.reset(new RobotContainer());
    }

    return g_rc.get();
}

RobotContainer::RobotContainer() 
  : ctrl(new t34::T34CommandXboxController(0))
  , swerve_drive(new t34::SwerveDrive())
  , DefaultCommand(swerve_drive, ctrl)
  , intake()
  , CenterOnCoralCommand(swerve_drive.get())
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 

{

  ctrl->POVRight().WhileTrue(std::move(CenterOnCoralCommand).ToPtr());
//Runs algae intake in on A button, spits out on B
  ctrl->A().OnTrue(intake.RunIn(-0.25));
  ctrl->B().OnTrue(intake.RunOut(0.7));

  //_________________________________________________

  //Runs coral intake in on X button, spits out on Y
  ctrl->X().OnTrue(coralintake.RunIn(-0.25));
  ctrl->Y().OnTrue(coralintake.RunOut(0.5));

  //Move the coral intake up and down on right bumper press
  ctrl->RightBumper().OnTrue(frc2::InstantCommand(
  [this]
    {
      if (coralintake.intakeflippedup) {
        coralintake.FlipArmDown();
        coralintake.intakeflippedup = false;
      } else {
        coralintake.FlipArmUp();
        coralintake.intakeflippedup = true;
      }
    }
  ).ToPtr()
  );

  //Moves the elevator up one level when the DPad up button is pressed
  ctrl->POVUp().OnTrue(elevator.MoveUpOnce());

  //Moves the elevator down one level when the DPad down button is pressed
  ctrl->POVDown().OnTrue(elevator.MoveDownOnce());

  //Moves the climber up and down on Left Bumber press
  ctrl->LeftBumper().OnTrue(frc2::InstantCommand(
    [this]
      {
        if (climber.climber_up)
        {
          climber.FlipArmDown();
          climber.climber_up = false;
        } else {
          climber.FlipArmUp();
          climber.climber_up = true;
        }
      }
    ).ToPtr()
    );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
