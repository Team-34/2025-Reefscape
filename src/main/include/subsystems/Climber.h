#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include "Talon.h"
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <frc/Servo.h>

using namespace ctre::phoenix6::hardware;

namespace t34
{   
    class Climber : public frc2::SubsystemBase 
    {
    public:
        Climber();
    
        frc2::CommandPtr Climb();
        frc2::CommandPtr RunArmBySpeed(double speed);
        frc2::CommandPtr RunLock(double position);
        
        void FlipLock();

        inline double GetLockPosition() { return m_lock.GetPosition(); }
        inline double GetArmPosition() { return m_motor.GetPosition().GetValueAsDouble(); }
    
    private:
        TalonFX m_motor;
    
        bool m_flipped_out;
        bool m_lock_flipped_up;

        frc::Servo m_lock;

    };
}