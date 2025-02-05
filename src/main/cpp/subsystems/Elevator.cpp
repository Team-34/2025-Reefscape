#include "subsystems/Elevator.h"

Elevator::Elevator()
//: m_motors(elevator_motor_id)
: m_vert_motor_left(999)
, m_vert_motor_right(1000)
, m_vert_motors_PID(0.5, 0.0, 0.0)
, m_wrist_motor(998, SparkLowLevel::MotorType::kBrushless)
, m_wrist_motor_PID(0.5, 0.0, 0.0)
, level(0)
{
    m_vert_motors_PID.SetTolerance(NEOUnitToInch(0.5));
    m_vert_motor_right.Follow(m_vert_motor_left);
}

frc2::CommandPtr Elevator::MoveWristToCommand(units::degree_t degree)
{
    return this->RunEnd([this, degree] {
        //m_motors_PID.Calculate(m_motors.GetSelectedSensorPosition(), height.value());
        m_wrist_motor.Set(m_wrist_motor_PID.Calculate(
            NEOUnitToDegree(m_wrist_motor.GetEncoder().GetPosition()), NEOUnitToDegree(degree.value())));
    },
    [this]{
        //m_motors.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        m_wrist_motor.StopMotor();

    }).Until([this]{
        return m_wrist_motor_PID.AtSetpoint();

    });
}

frc2::CommandPtr Elevator::ElevateCommand(units::inch_t height)
{

    return this->RunEnd([this, height] {
        //m_motors_PID.Calculate(m_motors.GetSelectedSensorPosition(), height.value());
        m_vert_motor_left.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_vert_motors_PID.Calculate(
            NEOUnitToInch(m_vert_motor_left.GetSelectedSensorPosition()), NEOUnitToInch(height.value())));
    },
    [this]{
        m_vert_motor_left.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }).Until([this]{
        return m_vert_motors_PID.AtSetpoint();
    });
}

frc2::CommandPtr Elevator::MoveToLevelCommand()
{
    switch (level)
    {
    case 0:
        return MoveWristToCommand(units::degree_t(85))
        .AndThen(ElevateCommand(units::inch_t(0)));
        break;

    case 1:
        return MoveWristToCommand(units::degree_t(35))
        .AndThen(ElevateCommand(units::inch_t(18 - BASE_HEIGHT_FROM_FLOOR_INCHES.value())));
        break;
        
    case 2:
        return MoveWristToCommand(units::degree_t(35))
        .AndThen(ElevateCommand(units::inch_t(31 + (7/8) - BASE_HEIGHT_FROM_FLOOR_INCHES.value())));
        break;
        
    case 3:
        return MoveWristToCommand(units::degree_t(35))
        .AndThen(ElevateCommand(units::inch_t(47 + (7/8) - BASE_HEIGHT_FROM_FLOOR_INCHES.value())));
        break;
        
    case 4:
        return MoveWristToCommand(units::degree_t(88))
        .AndThen(ElevateCommand(units::inch_t(72 - BASE_HEIGHT_FROM_FLOOR_INCHES.value())));
        break;
        
    }
}

frc2::CommandPtr Elevator::MoveUpOnceCommand() {
    return this->RunOnce([this]{
    level = (level < 4 ) ? level++ : level;
    });
}

frc2::CommandPtr Elevator::MoveDownOnceCommand() {
    return this->RunOnce([this]{
    level = (level > 0 ) ? level-- : level;
    });
}


