#include "Robot.h"
#include "constants/ElevatorConstants.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class Elevator
{
    private:

        double toflastSpeed = 0;
        double toflastHeight = 0;

        bool lastSensorValue = false;
        int timesPassed = 0;

        rev::CANSparkMax ampMotor;
        rev::SparkLimitSwitch ampMechSensor;
        
    public:

        ctre::phoenix6::hardware::TalonFX winchMotor;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints;
        frc::ProfiledPIDController<units::meters> m_controller;
        frc::ElevatorFeedforward m_feedforward;

        void BeginPIDElevator();
        bool PIDElevator(double setpoint);

        enum ElevatorSetting{LOW, INTAKE, OUTTAKE, AMP, TRAP};

        Elevator();

        void SetAmpMotorPercent(double percent);

        double GetWinchEncoderReading();

        void StopElevator();

        void MoveElevatorPercent(double percent);

        void ResetElevatorEncoder();

        bool GetObjectInMech();

        void NoteFromSelector();
        void NoteToSelector();
        void DepositNote();

        double GetElevatorSetpoint();



        bool GetElevatorAtSetpoint();

        bool MoveToHeight(ElevatorSetting Height);

};

#endif