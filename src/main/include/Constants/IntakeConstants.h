#ifndef INTAKE_CONSTANTS_H
#define INTAKE_CONSTANTS_H

// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define INTAKE_MOTOR_PORT 17 
#define FIXED_INTAKE_MOTOR_PORT 18
#define WRIST_MOTOR_PORT 19

//Intake motor speeds, percent
#define INTAKE_SPEED_IN 50.0
#define INTAKE_SPEED_OUT 50.0

//Wrist absolute Encoder position values, 0-1 rotatons
#define WRIST_HIGH 0.700
inline double WRIST_LOW = 0.975;

#define WRIST_SPEED_LOW_THRESHHOLD -2.0

//Wrist PID Constants
inline double WRISTFF  = 0;
inline double WRISTKP  = 2.1;
inline double WRISTKI  = 0.03;
inline double WRISTKIMAX  = 0.1;
inline double ALLOWABLE_ERROR_WRIST  = 0.007;
inline double WRISTMAX_SPEED  = 0.3;

#endif