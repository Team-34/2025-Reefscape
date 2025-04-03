#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Servo.h>

using namespace ctre::phoenix6::hardware;

namespace t34
{   
    class Climber : public frc2::SubsystemBase 
    {
    public:
        Climber();
    
        frc2::CommandPtr Climb();
        frc2::CommandPtr RunArm(double power);
        frc2::CommandPtr RunLock(double power);
        frc2::CommandPtr FlipLockCommand();

        frc2::Command

        void FlipLock();
        void SetLock();

        inline double GetLockPosition() { return m_lock.GetPosition(); }
        inline double GetArmPosition() { return m_motor.GetPosition().GetValueAsDouble(); }
    
    private:
        TalonFX m_motor;
    
        bool m_flipped_out;
        bool m_locked;
    
        frc::PIDController m_pid_controller;
        frc::Servo m_lock;
    };
}