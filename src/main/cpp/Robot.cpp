#include "Robot.h"
#include "subsystems/LimelightUtil.h"

Robot::Robot() {}

void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
  frc::SmartDashboard::PutNumber("Gyro Yaw", m_gyro->GetYaw().GetValueAsDouble());

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() 
{
  frc2::CommandScheduler::GetInstance().Schedule(rc.m_coordinator.ScoreL4Auto());
  // rc.m_coordinator.ScoreL3Auto().Unwrap()->Schedule();
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() 
{
  // rc.m_coordinator.ScoreL3Auto().Unwrap()->Cancel();
  frc2::CommandScheduler::GetInstance().CancelAll();
  frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc.swerve_drive.get(), std::move(rc.m_default_command));
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() 
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif
