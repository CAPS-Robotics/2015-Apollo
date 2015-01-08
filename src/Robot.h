#ifndef __MY_ROBOT_H_
#define __MY_ROBOT_H_

#include <pthread.h>

void* driveFunc(void* arg);

RobotDrive      *drive;
Joystick        *joystick;
Gyro            *gyro;

bool             driveRun;
pthread_t        driveThread;

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
