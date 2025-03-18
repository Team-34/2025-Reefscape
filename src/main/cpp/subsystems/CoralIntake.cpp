#include "subsystems/CoralIntake.h"

namespace t34
{    
  

  CoralIntake::CoralIntake()
    : m_motor(4, rev::spark::SparkLowLevel::MotorType::kBrushless)
    , m_config{}
  {
    m_config //New configs for Spark Maxes, i am using the default settings provided in the REV documentation. 
      .Inverted(true)
      .SetIdleMode(SparkMaxConfig::IdleMode::kBrake);
    m_config.encoder
      .PositionConversionFactor(1000)
      .VelocityConversionFactor(1000);
    m_config.closedLoop
      .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
      .Pid(1.0, 0.0, 0.0);

    m_motor.Configure(m_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
  } 

  frc2::CommandPtr CoralIntake::RunInCommand()
  {
    return this->RunEnd(
      [this] { this->m_motor.Set(-0.25); },
      [this] { this->m_motor.StopMotor(); }
    );
  }
  
  frc2::CommandPtr CoralIntake::RunOutCommand()
  {
    return this->RunEnd(
      [this] { this->m_motor.Set(0.5); },
      [this] { this->m_motor.StopMotor(); }
    );
  }
}