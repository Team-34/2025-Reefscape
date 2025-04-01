#include "subsystems/AlgaeIntake.h"
#include "Neo.h"
#include <frc/controller/PIDController.h>
#include "Constants.h"

namespace t34
{    
  AlgaeIntake::AlgaeIntake()
    : m_motor(5)
    , m_init_algae_angle(155_deg) //65 degrees away from horizontal
    , m_right_wrist_motor(1)
    , m_left_wrist_motor(2)
    , m_slot0Configs()
  {
    m_slot0Configs.kP = 2.4; 
    m_slot0Configs.kI = 0;
    m_slot0Configs.kD = 0.1;

    m_left_wrist_motor.GetConfigurator().Apply(m_slot0Configs);
    m_right_wrist_motor.GetConfigurator().Apply(m_slot0Configs);
  } 

  void AlgaeIntake::MoveWristTo(double enc_units)
  {
    m_left_wrist_motor.SetPosition(units::turn_t(enc_units));
    m_right_wrist_motor.SetPosition(units::turn_t(enc_units));
  }

  void AlgaeIntake::MoveWristTo(units::degree_t angle)
  {
    m_left_wrist_motor.SetPosition(units::turn_t(angle));
    m_right_wrist_motor.SetPosition(units::turn_t(angle));
  }

  frc2::CommandPtr AlgaeIntake::MoveWristToCommand(units::degree_t angle)
  {
    return this->RunEnd
    (
      [this, angle] 
      { 
        m_left_wrist_motor.SetPosition(units::turn_t(angle));
        m_right_wrist_motor.SetPosition(units::turn_t(angle));
      },
      [this]
      {
        m_left_wrist_motor.StopMotor();
        m_right_wrist_motor.StopMotor();
      }
    );
  }

 frc2::CommandPtr AlgaeIntake::MoveWristByPowerCommand(double val)
  {
    return this->RunEnd
    (
      [this, val] 
      { 
        m_left_wrist_motor.Set(val);
        m_right_wrist_motor.Set(val);
      },
      [this]
      {
        m_left_wrist_motor.StopMotor();
        m_right_wrist_motor.StopMotor();
      }
    );
  }
  void AlgaeIntake::Periodic() {}
}