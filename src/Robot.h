#ifndef __MY_ROBOT_H_
#define __MY_ROBOT_H_

typedef enum
{
    down = -1,
    stay = 0,
    up = 1
} Status;

void driveFunc();
void inputFunc();

RobotDrive      *drive;
Joystick        *joystick;
Talon			*liftTalon;
DoubleSolenoid	*shifter;
DoubleSolenoid	*claw;
Compressor		*compressor;
Gyro            *gyro;
AnalogInput		*stringPot;
DigitalInput	*topLimitSwitch;
DigitalInput	*botLimitSwitch;
DigitalOutput	*LED;

bool             driveRun;
std::thread      *driveThread;
std::thread		 *inputThread;

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

class Apollo : public IterativeRobot
{
public:
    Apollo() {};
    ~Apollo() {};

    void RobotInit();
    void AutonomousInit();
    void TeleopInit();
    void DisabledInit();

    // unused functions
    void TestInit() {}
    void DisabledPeriodic() {}
    void AutonomousPeriodic() {}
    void TeleopPeriodic() {}
    void TestPeriodic() {}
};

#endif
