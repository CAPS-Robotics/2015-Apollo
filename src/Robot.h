#ifndef __MY_ROBOT_H_
#define __MY_ROBOT_H_

#include <pthread.h>

typedef enum
{
    down = -1,
    stay = 0,
    up = 1
} Status;

void* driveFunc(void* arg);
void* inputFunc(void* arg);
void* macroFunc(void* arg);

RobotDrive      *drive;
Joystick        *joystick;
Talon			*liftTalon;
DoubleSolenoid	*shifter;
DoubleSolenoid	*claw;
Compressor		*compressor;
Gyro            *gyro;
DigitalInput	*topLimitSwitch;
DigitalInput	*botLimitSwitch;
DigitalInput	*carriageSwitch;

bool             driveRun;
pthread_t        driveThread;
pthread_t		 inputThread;
pthread_t		 macroThread;

float gyroAngle;

int height;
const int maxheight = 4;
Status motorStatus = stay;

int sgn(double num) {
	return num == 0 ? 0 : num / fabs(num);
}

float safe_motor(float power) {
	return fabs(power) > 1 ? sgn(power) : power;
}

class Seabiscuit : public IterativeRobot
{
public:
    Seabiscuit() {};
    ~Seabiscuit();

    void RobotInit();
    void TeleopInit();
    void DisabledInit();

    // unused functions
    void TestInit() {}
    void DisabledPeriodic() {}
    void AutonomousPeriodic() {}
    void AutonomousInit() {}
    void TeleopPeriodic() {}
    void TestPeriodic() {}
};

#endif
