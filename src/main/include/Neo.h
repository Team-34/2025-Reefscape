#pragma once

#include <units/angle.h>
#include <units/length.h>
#include "Constants.h"

namespace t34
{
  namespace Neo
  {
    constexpr auto NEO550_RESOLUTION         { 42.0 };
    constexpr auto NEO550_SHAFT_CIRCUMFERENCE{ units::inch_t{ 1.0 / 8.0 } * M_PI };

    inline constexpr double DegreeTo550Unit(units::degree_t degrees)
    {
        return (degrees / 360_deg) * NEO550_RESOLUTION;
    }

    inline constexpr double InchTo550Unit(units::inch_t inches)
    {
        return (inches / NEO550_SHAFT_CIRCUMFERENCE) * NEO550_RESOLUTION;
    }

  } // namespace Neo
} // namespace t34
