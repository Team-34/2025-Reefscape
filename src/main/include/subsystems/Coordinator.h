// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <subsystems/Elevator.h>

namespace t34
{
    class Coordinator : public frc2::SubsystemBase 
    {
public:
    Coordinator(t34::Elevator* elevator);

    void MoveToLevel(int level);

    

    inline frc2::CommandPtr MoveUpLevelCommand() { return this->RunOnce([this]{MoveToLevel(m_current_level + 1);}); }
    inline frc2::CommandPtr MoveDownLevelCommand() { return this->RunOnce([this]{MoveToLevel(m_current_level - 1);}); }

    void Periodic() override;

private:
    t34::Elevator* m_elevator;
    int m_current_level;
  
};
}
