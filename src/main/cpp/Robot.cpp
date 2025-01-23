// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <rev/SparkMax.h>
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"


Robot::Robot() {
  
  rc = RobotContainer::Get();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();
  }

  frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc->swerve_drive.get(), std::move(rc->DefaultCommand));
}

void Robot::TeleopPeriodic() {

  //Runs algae intake in on A button, spits out on B
  rc->ctrl->A().OnTrue(rc->intake.RunIn(-0.25));
  rc->ctrl->B().OnTrue(rc->intake.RunOut(0.7));
  
  //_________________________________________________

  //Runs coral intake in on X button, spits out on Y
  rc->ctrl->X().OnTrue(rc->coralintake.RunIn(-0.25));
  rc->ctrl->Y().OnTrue(rc->coralintake.RunOut(0.5));

  //Checks if the intakeflippedup bool is true. If it is, it flips it down and sets the bool to false. if it is false, it flips it up and sets it to true.
  rc->ctrl->RightBumper().OnTrue(frc2::InstantCommand(
  [this]
    {
      if (rc->coralintake.intakeflippedup) {
        rc->coralintake.FlipArmDown();
        rc->coralintake.intakeflippedup = false;
      } else {
        rc->coralintake.FlipArmUp();
        rc->coralintake.intakeflippedup = true;
      }
    }
  ).ToPtr()
  );


}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
