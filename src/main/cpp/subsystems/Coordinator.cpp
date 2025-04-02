#include "subsystems/Coordinator.h"

namespace t34{
Coordinator::Coordinator() {}

CommandPtr Coordinator::TestPathCommand(){
  return this->StartRun(
    [this] {},
    [this] {}

  ).AndThen(m_elevator.);
}

// This method will be called once per scheduler run
void Coordinator::Periodic() {}
}