#include "Constants/IntakeConstants.h"
#include "Robot.h"

class Intake
{
public:
<<<<<<< Updated upstream
    rev::CANSparkMax intakeMotor;
=======

    rev::CANSparkMax intakeMotor;
    rev::CANSparkMax wristMotor;
    rev::SparkAbsoluteEncoder *magEncoder;

    double runningWristIntegral = 0;
    double lastWristSpeed = 0;
>>>>>>> Stashed changes

    Intake();

    void SetIntakeMotorSpeed(double percent);
    void IntakeRing();
    void OuttakeRing();
<<<<<<< Updated upstream
=======

    double GetWristEncoderReading();

    void MoveWristPercent(double percent);

    bool PIDWrist(double point);

    bool PIDWristDown();

    bool PIDWristUp();

>>>>>>> Stashed changes
};