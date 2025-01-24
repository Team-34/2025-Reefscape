// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator()
//: m_motors(elevator_motor_id)
: m_vert_motors(elevator_motor_id, SparkLowLevel::MotorType::kBrushless)
, m_vert_motors_PID(0.5, 0.0, 0.0)
, m_wrist_motor(wrist_motor_id, SparkLowLevel::MotorType::kBrushless)
, m_wrist_motor_PID(0.5, 0.0, 0.0)
, level(0)
{
    m_vert_motors_PID.SetTolerance(NEOUnitToInch(0.5));

}

frc2::CommandPtr Elevator::MoveWristTo(units::degree_t degree)
{
    return this->StartEnd([this, degree] {
        //m_motors_PID.Calculate(m_motors.GetSelectedSensorPosition(), height.value());
        m_vert_motors.Set(m_wrist_motor_PID.Calculate(
            NEOUnitToDegree(m_wrist_motor.GetEncoder().GetPosition()), NEOUnitToDegree(degree.value())));

    },
    [this]{
        //m_motors.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_wrist_motor.Set(0.0);

    }).Until([this]{
        return m_wrist_motor_PID.AtSetpoint();

    });
}

frc2::CommandPtr Elevator::Elevate(units::inch_t height)
{

    return this->StartEnd([this, height] {
        //m_motors_PID.Calculate(m_motors.GetSelectedSensorPosition(), height.value());
        m_vert_motors.Set(m_vert_motors_PID.Calculate(
            NEOUnitToInch(m_vert_motors.GetEncoder().GetPosition()), NEOUnitToInch(height.value())));

    },
    [this]{
        //m_motors.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_vert_motors.Set(0.0);

    }).Until([this]{
        return m_vert_motors_PID.AtSetpoint();

    });
}

frc2::CommandPtr Elevator::MoveToLevel()
{
    return this->Run([this] {
        switch (level)
        {
        case 0:
            MoveWristTo(units::degree_t(85));
            Elevate(units::inch_t(0));
    
        case 1:
            MoveWristTo(units::degree_t(35));
            Elevate(units::inch_t(18));
            
        case 2:
            MoveWristTo(units::degree_t(35));
            Elevate(units::inch_t(31 + (7/8)));
            
        case 3:
            MoveWristTo(units::degree_t(35));
            Elevate(units::inch_t(47 + (7/8)));
            
        case 4:
            MoveWristTo(units::degree_t(88));
            Elevate(units::inch_t(72));
            
        }
    });
}

frc2::CommandPtr Elevator::MoveUpOnce() {
    level = (level < 5 ) ? level++ : level;
}

frc2::CommandPtr Elevator::MoveDownOnce() {
    level = (level > -1 ) ? level-- : level;
}


// This method will be called once per scheduler run
void Elevator::Periodic() {



}
