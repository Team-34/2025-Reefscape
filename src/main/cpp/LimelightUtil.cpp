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

    for (LimelightHelpers::RawFiducial at : m_apriltags)
    {
        if (at.distToRobot < result_at.distToRobot)
        {
            result_at = at;
        }
    }

    return result_at;
}

units::inch_t t34::LimelightUtil::CalcDistanceFromNearest()
{
    //LimelightHelpers::RawFiducial at = GetNearestAT();

    return units::inch_t( (log2(GetNearestAT().ta) - 3.0) * 2.54);

}

units::inch_t t34::LimelightUtil::CalcDistance()
{
    return units::inch_t( (log2(LimelightHelpers::getTA(m_nt_name)) - 3.0) * 2.54);
}