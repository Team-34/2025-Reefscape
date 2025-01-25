#include "Intake.h"
#include <frc/controller/PIDController.h>

using namespace rev::spark;

class CoralIntake : public t34::Intake {
public:

CoralIntake();

frc2::CommandPtr RunIn(double power_percentage);
frc2::CommandPtr RunOut(double power_percentage);
frc2::CommandPtr FlipArmUp();
frc2::CommandPtr FlipArmDown();

bool m_intake_flipped_up;

private:

SparkMax m_coral_wrist_motor;
SparkMax m_coral_intake_motor;

frc::PIDController m_motor_pid;

};