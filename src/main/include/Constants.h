#pragma once

#include <string>
#include <cmath>
#include <units/mass.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/current.h>
#include <frc/system/plant/DCMotor.h>
#include <vector>
#include <frc/geometry/Translation2d.h>

const int CONTROLLER_PORT = 0;

constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI2{ M_PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / M_PI };
constexpr double PI_DIV_180{ M_PI / 180.0 };
constexpr double NEO550_RES{ 42 };
constexpr double INTAKE_GEAR_RATIO{ 203.636364 };
constexpr double ARM_ENC_CONVERSION_FACTOR{ 360.0 / (NEO550_RES * INTAKE_GEAR_RATIO) };
constexpr double CLIMBER_UNITS_TO_INCHES_FACTOR{ 1 };
constexpr double ARM_DEG_SCALAR{ 0.02756 };
constexpr double SHOOTER_DEG_SCALAR{ 0.0116 };
constexpr double LIMELIGHT_DEGREE_SCALAR{ 23.188 / 20.25 };//{ 21.1726 / 22.5 };
constexpr double SHOOTER_OFFSET_ANGLE_DEG{ 59 };

const std::string LIMELIGHT_TABLE_NAME{ "" };

const int POV_UP{ 0 };
const int POV_RIGHT{ 90 };
const int POV_DOWN{ 180 };
const int POV_LEFT{ 270 };

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)

/*
Elevator Subsytem Constants
*/

const int ELEVATOR_MOTOR_ID{ 0 };

/*PathPlanner Constants

const units::kilogram_t ROBOT_WEIGHT{ 85.0 };
const units::kilogram_square_meter_t MOI{ 1.0 };
const int MOTOR_NUM{ 8 };
const std::vector<frc::Translation2d> MODULE_OFFSETS{ 
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //left aft drive
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //left aft steer
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //left front drive
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //left front steer
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //right aft drive
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //right aft steer
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))), //right front drive
    frc::Translation2d(units::meter_t(1), frc::Rotation2d(units::degree_t(0))) //right front steer
    };

const units::meter_t WHEEL_RADIUS{0.1};
const units::meters_per_second_t MAX_DRIVE_VELOCITY{ 15.0 };
const double WHEEL_COF{0.2};
const frc::DCMotor DRIVE_MOTOR;
const double DRIVE_GEARING{ 12.0 };
const units::ampere_t DRIVE_CURRENT_LIMIT{ 40.0 };*/