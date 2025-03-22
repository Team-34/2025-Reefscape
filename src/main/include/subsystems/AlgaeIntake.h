//uses sparkmax for now
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

#include "Talon.h"

using namespace ctre::phoenix6::hardware;
using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol::can;

namespace t34
{
    class AlgaeIntake : public frc2::SubsystemBase, public Intake 
    {
    public:
        AlgaeIntake();

        virtual frc2::CommandPtr RunInCommand() override;
        virtual frc2::CommandPtr RunOutCommand() override;

        void Periodic() override;

        frc2::CommandPtr MoveAlgaeWristToCommand(double enc_units);
        frc2::CommandPtr MoveAlgaeWristToCommand(units::degree_t angle);
        frc2::CommandPtr MoveAlgaeWristByPowerCommand(double val);
        frc2::CommandPtr MoveAlgaeWristByIncrementCommand(double increase);

        
    private:
        ctre::phoenix6::hardware::TalonFX m_motor;

        const units::degree_t m_init_algae_angle;

        TalonSRX m_right_algae_wrist_motor;
        TalonSRX m_left_algae_wrist_motor;

        frc::PIDController m_algae_wrist_pid;
        
    };
};