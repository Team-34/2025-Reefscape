// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LimelightUtil.h"

t34::LimelightUtil::LimelightUtil(std::string nt_name)
: m_nt_name(nt_name)
{

}

// This method will be called once per scheduler run
void t34::LimelightUtil::Periodic() 
{

}

units::inch_t t34::LimelightUtil::CalcDistance()
{
    double x = GetTA();
    
    /*
    *Trendline equation generated using Google Sheets- x: tA, y: ft
    *ex: 
    *
    *tA   | ft
    *     |
    *2.958| 3
    *2.301| 3.5
    *1.544| 4
    *1.266| 4.5
    *0.951| 5
    */
    double dist_equation = 5.13 * pow(x, -0.486); 

    //Arbitrary value derived from imperical testing
    double avg_error = (-0.04 * x) - 0.12;

    return units::inch_t( (dist_equation + avg_error) * 12.0 );
}