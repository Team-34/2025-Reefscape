// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <rev/SparkMax.h>


Robot::Robot() {
  
  rc = RobotContainer::Get();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutNumber("travelled", rc->AutoDrive.GetTravelled().value());
  frc::SmartDashboard::PutNumber("setpoint", rc->AutoDrive.GetSetpoint().value());
  frc::SmartDashboard::PutNumber("y", rc->AutoDrive.GetYTravelled().value());
  frc::SmartDashboard::PutNumber("x", rc->AutoDrive.GetXTravelled().value());
  frc::SmartDashboard::PutNumber("y output", rc->AutoDrive.GetYOutput());
  frc::SmartDashboard::PutNumber("x output", rc->AutoDrive.GetXOutput());
  frc::SmartDashboard::PutNumber("steer output", rc->AutoDrive.GetSteerOutput());
  frc::SmartDashboard::PutNumber("Setpoint + travelled", rc->AutoDrive.GetSetpoint().value() + rc->AutoDrive.GetTravelled().value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
//   m_autonomousCommand = rc->GetAutonomousCommand();

//   if (m_autonomousCommand) {
//     m_autonomousCommand->Schedule();
//   }
    rc->AutoDrive.Schedule();

    //rc->AutoDrive.Initialize();

}

void Robot::AutonomousPeriodic() {

  //rc->AutoDrive.Execute();
}

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