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

units::inch_t t34::LimelightUtil::CalcDistance()
{
    double x = GetTA();
    
    double dist_equation = 5.13 * pow(x, -0.486); 
    double avg_error = -0.04 * x -0.12;

    return units::inch_t( (dist_equation + avg_error) * 12.0 );
}