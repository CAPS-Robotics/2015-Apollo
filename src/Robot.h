#ifndef __MY_ROBOT_H_
#define __MY_ROBOT_H_

#include <pthread.h>

void* driveFunc(void* arg);

RobotDrive      *drive;
Joystick        *joystick;

bool             driveRun;
pthread_t        driveThread;

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
