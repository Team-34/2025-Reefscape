// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <iostream>
#include <vector>
#include <units/length.h>
//#include <units/angle.h>
#include <cmath>

#include "LimelightHelpers.h"

namespace t34
{

  class LimelightUtil : public frc2::SubsystemBase 
  {
  public:

    LimelightUtil(std::string nt_name);

    void Periodic() override;
    
    /*Calculates the distance between the center of a detected Limelight object and the camera*/
    units::inch_t CalcDistance();

    /*Returns the tracked object's middle x-coordinate*/
    inline double GetTX() { return LimelightHelpers::getTX(m_nt_name); }

    /*Returns the tracked object's middle y-coordinate*/
    inline double GetTY() { return LimelightHelpers::getTY(m_nt_name); }

    /*Returns the tracked object's total area*/
    inline double GetTA() { return LimelightHelpers::getTA(m_nt_name); }

    /*Returns the tracked object's ID*/
    inline double GetID() { return LimelightHelpers::getFiducialID(m_nt_name); }

    /*Returns true if Limelight is detecting a valid object*/
    inline bool IsDetecting() { return LimelightHelpers::getTV(m_nt_name); }


  private:

    std::string m_nt_name;
  };


}