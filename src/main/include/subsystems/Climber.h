#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"

using namespace rev::spark;

class Climber : public frc2::SubsystemBase {

    public:

    Climber();

    frc2::CommandPtr FlipArmUp();
    frc2::CommandPtr FlipArmDown();

    bool climber_flipped_up;

    private:

    frc::PIDController m_pid_controller;

    SparkMax m_motor;

};