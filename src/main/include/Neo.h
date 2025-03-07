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

    // inline constexpr double DegreeTo550Unit(units::degree_t degrees)
    // {
    //   return degrees * NEO550_RESOLUTION;
    // }

    // inline constexpr double InchTo550Unit(units::inch_t inches)
    // {   
    //   return DegreeTo550Unit(inches / NEO550_SHAFT_CIRCUMFERENCE);
    // }

  } // namespace Neo
} // namespace t34
