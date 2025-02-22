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

    inline constexpr double DegreeTo550Unit(units::degree_t degrees)
    {
      return degrees * NEO550_RESOLUTION;
    }

    inline constexpr double InchTo550Unit(units::inch_t inches)
    {   
      return DegreeTo550Unit(inches / NEO550_SHAFT_CIRCUMFERENCE);
    }

  } // namespace Neo
} // namespace t34
