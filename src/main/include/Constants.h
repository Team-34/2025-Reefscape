#pragma once

#include <string>
#include <cmath>
#include <units/length.h>
#include <units/angle.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846	/* pi */
#endif
#ifndef M_PIl
#define M_PIl 3.141592653589793238462643383279502884L /* pi */
#endif

const int CONTROLLER_PORT = 0;

constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI2{ M_PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / M_PI };
constexpr double PI_DIV_180{ M_PI / 180.0 };
constexpr double NEO550_RES{ 42 };
constexpr double NEO550_SHAFT_CIRCUMFERENCE_INCH{ 0.125 * M_PI };
constexpr double LIMELIGHT_DEGREE_SCALAR{ 23.188 / 20.25 };//{ 21.1726 / 22.5 };
const units::inch_t BASE_HEIGHT_FROM_FLOOR{ 2.0 };

//const std::string LIMELIGHT_TABLE_NAME{ "" };

/**
 * Converts inches into NEO550 encoder units.
 * 
 * @returns inches * (1/NEO550_SHAFT_CIRCUMFERENCE_INCH) * NEO550_RES
 */
inline double InchToNEOUnit(units::inch_t inches)
{
// 42 encoder units = ~0.3927 inches
// 1 inch = 1/NEO550_SHAFT_CIRCUMFERENCE_INCH = ~2.546 NEO550 revolutions
// 1 inch = (1/NEO550_SHAFT_CIRCUMFERENCE_INCH) * NEO550_RES = ~106.952 encoder units

    return (1.0/NEO550_SHAFT_CIRCUMFERENCE_INCH) * NEO550_RES * inches.value();
}

/**
 * Converts degrees into NEO550 encoder units.
 * 
 * @returns (NEO550_RES / 360) * degrees
 */
inline double DegreeToNEOUnit(units::degree_t degrees)
{
    return (NEO550_RES / 360.0) * degrees.value();
}

#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)