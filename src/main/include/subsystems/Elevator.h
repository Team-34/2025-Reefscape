#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <frc/AnalogEncoder.h>
#include "Constants.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/AlgaeIntake.h"

using namespace ctre::phoenix::motorcontrol::can;

namespace t34
{

  class Elevator : public frc2::SubsystemBase
  {
  public:

    Elevator();

    void ElevateTo(units::inch_t height);
    void ElevateTo(double height);
    void Stop();

    frc2::CommandPtr ElevateToCommand(units::inch_t height);
    frc2::CommandPtr ElevateToCommand(double height);
    frc2::CommandPtr ElevateToEncValue(double enc_value);

    units::inch_t GetPosition();

    inline bool EndCondition() { return m_pid.AtSetpoint(); };

    //frc2::CommandPtr MoveToRestCommand();

    frc2::CommandPtr MoveElevatorByPowerCommand(double val);    
    
    inline double GetPositionAsEncVal() { return m_encoder_accumulation; }

    void Periodic() override;

    double UpdatePosition(double acc, double last, double next);

  private:

    int m_level;

    double m_last_reading;

    //The USDigital MA3 encoder gets a looped 0-1 value, so it's required to get the total units
    double m_encoder_accumulation; 

    const units::inch_t m_init_height;

    TalonSRX m_left_motor;
    TalonSRX m_right_motor;

    frc::PIDController m_pid;
    frc::AnalogEncoder m_encoder;
  };
}
