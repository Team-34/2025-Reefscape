// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LimelightUtil.h"

t34::LimelightUtil::LimelightUtil(std::string nt_name, units::degree_t camera_angle)
: m_nt_name(nt_name)
, m_apriltags(LimelightHelpers::getRawFiducials(m_nt_name))
, m_tX(LimelightHelpers::getTX(m_nt_name))
, m_tY(LimelightHelpers::getTX(m_nt_name))
, m_tA(LimelightHelpers::getTX(m_nt_name))
, m_angle(camera_angle)
{
    m_apriltags = LimelightHelpers::getRawFiducials(m_nt_name);
}

// This method will be called once per scheduler run
void t34::LimelightUtil::Periodic() 
{

    //Update variables from network table
    m_apriltags = LimelightHelpers::getRawFiducials(m_nt_name);

    m_tX = LimelightHelpers::getTX(m_nt_name);
    m_tY = LimelightHelpers::getTY(m_nt_name);
    m_tA = LimelightHelpers::getTA(m_nt_name);
}

LimelightHelpers::RawFiducial t34::LimelightUtil::GetNearestAT()
{
    LimelightHelpers::RawFiducial result_at(m_apriltags[0]);
    if (m_apriltags.size() > 1) {

        for (LimelightHelpers::RawFiducial at : m_apriltags)
        {
            if (at.distToRobot < result_at.distToRobot)
            {
               result_at = at;
            }
        }
        return result_at;
    }
    

    return LimelightHelpers::RawFiducial(-1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

units::inch_t t34::LimelightUtil::CalcDistanceFromNearest()
{
    //LimelightHelpers::RawFiducial at = GetNearestAT();

    return units::inch_t( (log2(GetNearestAT().ta) - 3.0) * 2.54);

}

units::inch_t t34::LimelightUtil::CalcDistance()
{
    double x = GetTA();

    //double dist_equation = 2.62 + (-0.497 * x) + (0.0267 * std::pow(x, 2));

    //dist_equation:  183 + -261x + 162x^2 + -57.2x^3 + 12.8x^4 + -1.89x^5 + 0.188x^6 + -0.0125x^7 + 5.27E-04x^8 + -1.29E-05x^9 + 1.39E-07x^10
    double dist_equation = 183 + (-261 * x) + (162 * pow(x, 2)) + (-57.2 * pow(x, 3)) + (12.8 * pow(x, 4)) + (-1.89 * pow(x, 5)) + (0.188 * pow(x, 6)) + 
    (-0.0125 * pow(x, 7)) + (5.27 * E_(-4) * pow(x, 8)) + (-1.29 * E_(-5) * pow(x, 9)) + (1.39 * E_(-7) * pow(x, 10));

    //y_error:  10.7 + -0.617x + 1.48E-03x^2 + 1.15E-03x^3 + 4.59E-04x^4 + 7.61E-06x^5 + -2.06E-06x^6
    double y_error = 10.7 + (-0.617 * x) + (E_(-3) * pow(x, 2)) + (1.15 * E_(-3) * pow(x, 2)) + (4.59 * E_(-4) * pow(x, 4))
        + (7.61 * E_(-6) * pow(x, 5)) + (-2.06 * E_(-6) * pow(x, 6));

    //scalar:  5.23 + -2.7x + 0.659x^2 + -0.0832x^3 + 5.75E-03x^4 + -2.05E-04x^5 + 2.91E-06x^6
    double scalar = 5.23 + (-2.7 * x) + (-0.0659 * pow(x, 2)) + (-0.0832 * pow(x, 3)) + (5.75 * E_(-3) * pow(x, 4)) + (-2.05 * E_(-4) * pow(x, 5)) + (2.91 * E_(-6) * pow(x, 6));

    

    return units::inch_t( (dist_equation / y_error));
}