#pragma once

#include <units/angle.h>
#include <units/length.h>
#include "Constants.h"

namespace t34
{
  namespace Talon
  {
    constexpr auto SRX_RESOLUTION{ 4096.0 / 1_tr };

    constexpr auto _775PRO_SHAFT_CIRCUMFERENCE{ (M_PI * 0.197_in) / 1_tr };

    /**
     * Converts inches into 775PRO encoder units.
     * 
     * @returns length * (1/_775PRO_SHAFT_CIRCUMFERENCE_INCH) * SRX_RESOLUTION
     */
    inline double LengthToSRXUnit(units::inch_t length)
    {
      return (1.0/_775PRO_SHAFT_CIRCUMFERENCE) * SRX_RESOLUTION * length;
    }

    /**
     * Converts degrees into 775PRO encoder units.
     * 
     * @returns (SRX_RESOLUTION / 360) * angles
     */
    inline double AngleToSRXUnit(units::degree_t angle)
    {
      return SRX_RESOLUTION * angle;
    }
  } // namespace Talon
} // namespace t34
