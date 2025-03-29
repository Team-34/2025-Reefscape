#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_motor(5)
    , m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_wrist_motor(1, rev::spark::SparkLowLevel::MotorType::kBrushless)
    , m_left_wrist_motor(2, rev::spark::SparkLowLevel::MotorType::kBrushless)
  {

  } 

  void AlgaeIntake::MoveWristTo(double enc_units)
  {
    m_left_wrist_motor.GetClosedLoopController().SetReference(enc_units, rev::spark::SparkLowLevel::ControlType::kPosition);
    m_right_wrist_motor.GetClosedLoopController().SetReference(enc_units, rev::spark::SparkLowLevel::ControlType::kPosition);

  }

  void AlgaeIntake::MoveWristTo(units::degree_t angle)
  {
    m_left_wrist_motor.GetClosedLoopController().SetReference(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    m_right_wrist_motor.GetClosedLoopController().SetReference(angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
  }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(units::degree_t angle)
  {
    return this->RunOnce
    (
      [this, angle] 
        { 
        m_left_wrist_motor.GetClosedLoopController().SetReference( angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
        m_right_wrist_motor.GetClosedLoopController().SetReference( angle.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    );

  }

 frc2::CommandPtr AlgaeIntake::MoveWristByPowerCommand(double val)
  {
    return this->RunOnce
    (
      [this, val] 
        { 
        m_left_wrist_motor.GetClosedLoopController().SetReference( val, rev::spark::SparkLowLevel::ControlType::kVelocity);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));
        m_right_wrist_motor.GetClosedLoopController().SetReference( -val, rev::spark::SparkLowLevel::ControlType::kVelocity);//BOTH_WRIST_GEAR_RATIO * ((angle - m_init_coral_angle) / 1_tr));
        }
    );
  }

  void AlgaeIntake::Periodic()
  {
  // frc::SmartDashboard::PutNumber("Left Algae Wrist Encoder", m_left_wrist_motor.GetAbsoluteEncoder());
  //frc::SmartDashboard::PutNumber("Right Algae Wrist Encoder", m_right_wrist_motor.GetAbsoluteEncoder());
  }

}