#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>

using namespace rev::spark;

class Climber : public frc2::SubsystemBase {
    public :

    Climber();

    frc2::CommandPtr FlipArmCommand();

    SparkMax m_motor;

    private:

    bool m_engaged;

    frc::PIDController m_pid_controller;


};