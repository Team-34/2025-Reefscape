// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() 
  : m_swerve(new t34::SwerveDrive())
  , m_ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(m_swerve, m_ctrl)
  
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  //A and B run the intake in (intaking a piece) and out (spitting a piece out)
  m_ctrl->A().OnTrue(m_al_intake.RunInCommand());
  m_ctrl->B().OnTrue(m_al_intake.RunOutCommand());

  //X and Y run the intake in (intaking a piece) and out (spitting a piece out)
  m_ctrl->X().OnTrue(m_co_intake.RunInCommand());
  m_ctrl->Y().OnTrue(m_co_intake.RunOutCommand());

  //POV_UP moves the elevator up ONE level, POV_DOWN moves it down ONE level
  m_ctrl->POVUp().OnTrue(m_elevator.MoveUpOnceCommand());
  m_ctrl->POVDown().OnTrue(m_elevator.MoveDownOnceCommand());

  //Right bumper switches the state of the climber. If the climber is engaged, it disengages it, and the same vice-versa.
  //Robot should start with the climber DISENGAGED. 
  m_ctrl->RightBumper().OnTrue(m_climber.FlipArmCommand());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
