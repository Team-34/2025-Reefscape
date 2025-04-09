#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <rev/config/SparkMaxConfig.h>
#include "Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/controller/PIDController.h>
#include <string>
#include <cmath>
#include <units/length.h>
#include <units/angle.h>
#include <rev/SparkMax.h>
#include "Talon.h"

using namespace ctre::phoenix6::hardware;
using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol::can;

namespace t34
{
    class AlgaeIntake : public frc2::SubsystemBase
    {
    public:
        AlgaeIntake();

        void Periodic() override;

        frc2::CommandPtr MoveWristByPowerCommand(double val);

        frc2::CommandPtr MoveWristToCommand(units::degree_t angle);
        frc2::CommandPtr MoveWristToCommand(double setpoint);

        frc2::CommandPtr MoveWristByIncrementCommand(double increase);
        frc2::CommandPtr RunInCommand(double speed);
        frc2::CommandPtr RunOutCommand(double speed);

        void MoveWristTo(double enc_units);
        void MoveWristTo(units::degree_t angle);
        
    private:

        const units::degree_t m_init_algae_angle;

        frc::PIDController m_pid;
        
        TalonFX m_intake_motor;
        TalonSRX m_right_wrist_motor;
        TalonSRX m_left_wrist_motor;

        double m_encoder_units;
    };
};