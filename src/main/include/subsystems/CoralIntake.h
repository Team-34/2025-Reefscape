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

        frc2::CommandPtr RunInCommand();
        frc2::CommandPtr RunOutCommand();
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
        
    private:
        bool m_run_up;

        double m_encoder_setpoint;

        int m_coral_level;

        const units::degree_t m_init_coral_angle;

        SparkMax m_wrist_motor;
        SparkMax m_motor;
   };
}