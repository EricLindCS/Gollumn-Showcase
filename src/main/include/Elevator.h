#include "Robot.h"
#include "constants/ElevatorConstants.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class Elevator
{
    public:

    double toflastSpeed = 0;
    double toflastHeight = 0;

    enum ElevatorSetting{LOW, AMP, TRAP};

    rev::CANSparkMax winchMotor;
    rev::SparkRelativeEncoder *winchEncoder;
    rev::CANSparkMax ampMotor;
    frc::DigitalInput ampMechSensor;

    frc::TrapezoidProfile<units::meters>::Constraints m_constraints;
    frc::ProfiledPIDController<units::meters> m_controller;
    frc::ElevatorFeedforward m_feedforward;

    Elevator();

    void ResetElevatorEncoder();

    void SetAmpMotorPercent(double percent);

    double GetWinchEncoderReading();

    void StopElevator();

    void MoveElevatorPercent(double percent);

    bool PIDElevator(double setpoint);

    bool GetObjectInMech();

    bool GetElevatorAtSetpoint();

    bool MoveToHeight(ElevatorSetting Height);

};

#endif