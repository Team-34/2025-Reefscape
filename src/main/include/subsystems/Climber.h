#pragma once
#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "Talon.h"

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
        frc2::CommandPtr LockCommand();
        frc2::CommandPtr UnlockCommand();

        void Periodic() override;
        
       // void SetLock();

        inline double GetLockPosition() { return m_lock.GetPosition(); }
        inline units::turn_t GetArmPosition() { return m_motor.GetPosition().GetValue(); }
        inline bool AtLockingPosition() { return GetArmPosition() < 1_tr; }
        inline void Lock() 
        { 
            m_lock.Set(0.25); 
            m_motor.StopMotor(); 
        }
        inline void Unlock() { m_lock.Set(0.65);}
    
    private:
        TalonFX m_motor;
    
        bool m_flipped_out;
        bool m_lock_flipped_up;

        frc::Servo m_lock;

        frc2::Trigger m_at_locking_position;

    };
}