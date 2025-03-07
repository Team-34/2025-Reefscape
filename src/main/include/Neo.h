#pragma once

#include <units/angle.h>
#include <units/length.h>
#include "Constants.h"

namespace t34
{
  namespace Neo
  {
    constexpr auto NEO550_RESOLUTION         { 42.0 / 1_tr };
    constexpr auto NEO550_SHAFT_CIRCUMFERENCE{ (( 1.0_in / 8.0 ) * M_PI) / 1_tr };

    /**
     * Converts inches into NEO550 encoder units.
     * 
     * @returns length * (1/NEO550_SHAFT_CIRCUMFERENCE_INCH) * NEO550_RES
     */
    inline double LengthToNEOUnit(units::inch_t length)
    {
        // 42 encoder units = ~0.3927 inches
        // 1 inch = 1/NEO550_SHAFT_CIRCUMFERENCE_INCH = ~2.546 NEO550 revolutions
        // 1 inch = (1/NEO550_SHAFT_CIRCUMFERENCE_INCH) * NEO550_RES = ~106.952 encoder units

        return (1.0/NEO550_SHAFT_CIRCUMFERENCE) * NEO550_RESOLUTION * length;
    }

    /**
     * Converts degrees into NEO550 encoder units.
     * 
     * @returns (NEO550_RES / 360) * angle
     */
    inline double AngleToNEOUnit(units::degree_t angle)
    {
        return NEO550_RESOLUTION * angle;
    }
  } // namespace Neo
} // namespace t34
