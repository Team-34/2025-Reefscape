#include "subsystems/CoralIntake.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include "Constants.h"

CoralIntake::CoralIntake() 
: coralwristmotor(3, rev::spark::SparkLowLevel::MotorType::kBrushless)
, coralintakemotor(4, rev::spark::SparkLowLevel::MotorType::kBrushless)
, m_motor_pid(0.5, 0.5, 0.5)
, intakeflippedup(true)
{}

frc2::CommandPtr CoralIntake::RunIn(double power_percentage) {
    return this->StartEnd(
      [this, power_percentage] {
        this->coralintakemotor.Set(power_percentage); 
    },
      [this, power_percentage] {
        this->coralintakemotor.Set(0.0);
    }
    );
}

frc2::CommandPtr CoralIntake::RunOut(double power_percentage)
{
    return this->StartEnd(
      [this, power_percentage] {
        this->coralintakemotor.Set(power_percentage);
      },
      [this, power_percentage] {
        this->coralintakemotor.Set(0.0);
      }
    );
}

frc2::CommandPtr CoralIntake::FlipArmUp() 
{
    return this->StartEnd(
        [this] {
            m_motor_pid.SetSetpoint(NEOUnitToDegree(90));
        },
        [this] {
           coralwristmotor.Set(0.0); 
        }
    ).Until(
        [this] {
            return m_motor_pid.AtSetpoint();
        }
    );
}

frc2::CommandPtr CoralIntake::FlipArmDown() 
{
    return this->StartEnd(
        [this] {
            m_motor_pid.SetSetpoint(NEOUnitToDegree(0));
        },
        [this] {
           coralwristmotor.Set(0.0); 
        }
    ).Until(
        [this] {
            return m_motor_pid.AtSetpoint();
        }
    );
}

