#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
#include "Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/hardware/DeviceIdentifier.hpp>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <string>
#include <cmath>
#include <units/length.h>
#include <units/angle.h>
#include <frc/DigitalInput.h>

using namespace rev::spark;
using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix::motorcontrol::can;

namespace t34
{
    class CoralIntake : public frc2::SubsystemBase
    {
    public:
        CoralIntake();

        void Periodic() override;

        inline bool AtTopLimit() const { return !m_top_limit.Get(); }

        frc2::CommandPtr RunInCommand(double speed);
        frc2::CommandPtr RunOutCommand(double speed);
        frc2::CommandPtr MoveWristToCommand(units::degree_t angle);
        frc2::CommandPtr MoveWristToCommand(double encoder_units);

        void MoveWristTo(units::degree_t angle);
        void MoveWristTo(double encoder_units);
        void StopWrist();

        bool EndCondition(); 

        frc2::CommandPtr MoveWristByPowerCommand(double val);
        frc2::CommandPtr MoveToLevelCommand(int level);
        frc2::CommandPtr IncrementUp();
        frc2::CommandPtr IncrementDown();

        frc2::CommandPtr MoveToZero();
        
    private:

        frc::DigitalInput m_top_limit;

        SparkMax m_motor;

        int m_coral_level;

        const units::degree_t m_init_coral_angle;

        SparkMax m_wrist_motor;

        bool m_run_up;
        bool m_returning;

        double m_encoder_setpoint;

        SparkMaxConfig m_config{};

        
   };
}