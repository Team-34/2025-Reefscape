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

  frc::SmartDashboard::PutNumber("Drive enc units", rc->swerve_drive->GetModulePositions()[0].distance.value());
  frc::SmartDashboard::PutNumber("Drive setpoint", rc->AutoDrive.GetYPID().GetSetpoint());
  frc::SmartDashboard::PutNumber("Drive y error", rc->AutoDrive.GetYPID().GetError());
  frc::SmartDashboard::PutBoolean("AtSetpoint?", rc->AutoDrive.GetYPID().AtSetpoint());
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