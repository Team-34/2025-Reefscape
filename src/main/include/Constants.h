#pragma once

#include <string>
#include <cmath>
#include <units/length.h>

const int CONTROLLER_PORT = 0;

constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI2{ M_PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / M_PI };
constexpr double PI_DIV_180{ M_PI / 180.0 };
constexpr double NEO550_RES{ 42 };
constexpr double NEO550_SHAFT_CIRCUMFERENCE_INCH{ 0.125 * M_PI };
constexpr double LIMELIGHT_DEGREE_SCALAR{ 23.188 / 20.25 };//{ 21.1726 / 22.5 };

const units::inch_t BASE_HEIGHT_FROM_FLOOR_INCHES{ 2.0 };

//const std::string LIMELIGHT_TABLE_NAME{ "" };

inline double NEOUnitToInch(double units)
{
    return (units / NEO550_RES) * NEO550_SHAFT_CIRCUMFERENCE_INCH;
}

inline double NEOUnitToDegree(double units)
{
    return (NEO550_RES / 360.0) * units;
}

//const int POV_UP{ 0 };
//const int POV_RIGHT{ 90 };
//const int POV_DOWN{ 180 };
//const int POV_LEFT{ 270 };

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)

/*
Elevator Subsytem Constants
*/

const int elevator_motor_id{ 0 };
const int wrist_motor_id{ 0 };