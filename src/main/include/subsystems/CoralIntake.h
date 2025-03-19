//uses sparkmax for now
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
    class CoralIntake : public frc2::SubsystemBase, Intake 
    {
    public:
        CoralIntake();

        void Periodic() override;

        virtual frc2::CommandPtr RunInCommand() override;
        virtual frc2::CommandPtr RunOutCommand() override;

        frc2::CommandPtr MoveCoralWristToCommand(units::degree_t angle);
        frc2::CommandPtr MoveCoralWristToCommand(double encoder_units);

        frc2::CommandPtr MoveCoralWristByPowerCommand(double val);

        frc2::CommandPtr MoveToCoralLevelCommand(int level);

        frc2::CommandPtr IncrementCoralUp();
        frc2::CommandPtr IncrementCoralDown();
        
        frc2::CommandPtr ElevateCoralWristToCommand(int movelevelby);
        
    private:
        SparkMax m_motor;

        int m_coral_level;

        const units::degree_t m_init_coral_angle;

        SparkMax m_coral_wrist_motor;

        frc::PIDController m_coral_wrist_pid;
    };
}