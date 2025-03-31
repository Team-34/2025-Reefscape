// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Coordinator.h"
namespace t34{
Coordinator::Coordinator() {}

CommandPtr Coordinator::TestPathCommand(){
  return this->StartRun(
    [this] {},
    [this] {}

  ).AndThen(rc.m_elevator.MoveToLevelCommand(3));
}

// This method will be called once per scheduler run
void Coordinator::Periodic() {}
}