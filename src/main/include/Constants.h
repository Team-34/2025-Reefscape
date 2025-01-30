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
constexpr double LIMELIGHT_DEGREE_SCALAR{ 23.188 / 20.25 };
constexpr double CLAW_GEAR_RATIO{ 0.0112121212};

const units::inch_t BASE_HEIGHT_FROM_FLOOR_INCHES{ 2.0 };

inline double NEOUnitToInch(double units)
{
    return (units / NEO550_RES) * NEO550_SHAFT_CIRCUMFERENCE_INCH;
}

inline double NEOUnitToDegree(double units)
{
    return (NEO550_RES / 360.0) * units;
}

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)

/*
Elevator Subsytem Constants
*/

const int elevator_motor_id{ 19 };
const int wrist_motor_id{ 21 };