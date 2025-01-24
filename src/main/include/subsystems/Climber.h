#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>

using namespace rev::spark;

class Climber : public frc2::SubsystemBase {

    Climber();

    frc2::CommandPtr FlipArmUp();
    frc2::CommandPtr FlipArmDown();


    private:

    frc::PIDController m_pid_controller;

    SparkMax m_motor;

};