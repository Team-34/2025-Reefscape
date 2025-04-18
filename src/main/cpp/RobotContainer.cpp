#include "RobotContainer.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/CoralIntake.h"

RobotContainer::RobotContainer() 
  : swerve_drive(new t34::SwerveDrive())
  , ctrl(new t34::T34CommandXboxController(0))
  , m_default_command(swerve_drive, ctrl)
  , m_algae_intake()
  , m_coral_intake()
  , m_climber()
  , m_elevator()
  , m_coordinator(&m_elevator, &m_coral_intake, swerve_drive)
  , m_auto_leave(swerve_drive, 0_in, 3_ft, 0_deg)
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  ctrl->LeftStick() //Left underside button toggles Faris Mode
    .OnTrue(swerve_drive->EnableFarisModeCommand())
    .OnFalse(swerve_drive->DisableFarisModeCommand()); 

  ctrl->RightTrigger(0.75).OnTrue(m_climber.Climb()); //Right trigger deploys/retracts climb
//_Algae_Wrist_Controls_________________________________________________________________________________________________

(ctrl->RightBumper() && ctrl->LeftStick())
  .WhileTrue(m_algae_intake.MoveWristByPowerCommand(0.3)); //Right bumper pulls up the algae intake

(ctrl->LeftBumper() && ctrl->LeftStick())
  .WhileTrue(m_algae_intake.MoveWristByPowerCommand(-0.3)); //Left bumper pulls down the algae intake

(ctrl->LeftBumper() && !ctrl->LeftStick()) 
    .OnTrue(m_algae_intake.MoveWristToCommand(-1572));

(ctrl->RightBumper() && !ctrl->LeftStick()) 
    .OnTrue(m_algae_intake.MoveWristToCommand(6089));  

//_Elevator_Controls____________________________________________________________________________________________________

  (ctrl->POVDown() && ctrl->LeftStick()) //POV Down + left underside button shaves elevator down
    .WhileTrue(m_elevator.MoveElevatorByPowerCommand(-0.3));

  (ctrl->POVUp() && ctrl->LeftStick()) //POV Up + left underside button shaves elevator up
    .WhileTrue(m_elevator.MoveElevatorByPowerCommand(0.3));

  (ctrl->POVDown() && !ctrl->LeftStick()) //POV Down decrements the robot's level
    .OnTrue(m_coordinator.MoveDownLevelCommand());

  (ctrl->POVUp() && !ctrl->LeftStick()) //POV Up increments the robot's level
    .OnTrue(m_coordinator.MoveUpLevelCommand());  

//_Coral_Wrist_Controls_________________________________________________________________________________________________

  (ctrl->POVRight() && ctrl->LeftStick())  //POV Right + left underside button shaves coral intake up
    .WhileTrue(m_coral_intake.MoveWristByPowerCommand(0.25));

  (ctrl->POVLeft() && ctrl->LeftStick()) //POV Left + left underside button shaves coral intake down
    .OnTrue(m_elevator.ElevateToCommand(0.44).AndThen(m_coral_intake.MoveWristToCommand(4.83)));

  (ctrl->POVRight() && !ctrl->LeftStick()) //POV Right pulols the coral intake into scoring position
    .OnTrue(m_coral_intake.MoveWristToCommand(12.0));

  (ctrl->POVLeft() && !ctrl->LeftStick()) //POV Left pulls the coral intake back up
    .OnTrue(m_coral_intake.MoveToZero());

//_Intake_Controls______________________________________________________________________________________________________
  ctrl->Back().OnTrue(swerve_drive->ZeroYawCommand()); //Back button zeros the robot's gyro

  ctrl->A().WhileTrue(m_algae_intake.RunInCommand(0.5)); //A sucks in algae
  ctrl->B().WhileTrue(m_algae_intake.RunOutCommand(0.75)); //A spits out algae
  
  ctrl->X().WhileTrue(m_coral_intake.RunInCommand(0.25)); //X sucks in coral
  ctrl->Y().WhileTrue(m_coral_intake.RunOutCommand(0.5)); //Y spits out coral
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return frc2::cmd::RunEnd([this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, -1.0_m), 0.0);
  },
  [this]
  {
    swerve_drive->Drive(frc::Translation2d(0_m, 0_m), 0.0);
  });
}