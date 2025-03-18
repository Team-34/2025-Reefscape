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

const int BOTH_WRIST_GEAR_RATIO = 70; // 1:70
const int ELEVATOR_WINCH_GEAR_RATIO = 50; // 1:50

constexpr bool INVERT_GYRO{ true }; // Always ensure Gyro is CCW+ CW-

constexpr double PI2{ M_PI * 2.0 };
constexpr double _180_DIV_PI{ 180.0 / M_PI };
constexpr double PI_DIV_180{ M_PI / 180.0 };

const units::inch_t BASE_HEIGHT_FROM_FLOOR{ 2.0 };
const units::inch_t ELEVATOR_LOWEST_POINT_FROM_BASE{ 7.0 };

//const std::string LIMELIGHT_TABLE_NAME{ "" };


#define DEG_TO_RAD(x) (x * PI_DIV_180)
#define RAD_TO_DEG(x) (x * _180_DIV_PI)