#include "WPILib.h"
#include "Robot.h"
#include "config.h"

Seabiscuit::~Seabiscuit()
{
    delete drive;
    delete joystick;
}

void Seabiscuit::RobotInit()
{
    drive = new RobotDrive(LEFT_MOTOR_PWM, RIGHT_MOTOR_PWM);
    joystick = new Joystick(JOY_PORT_1);

    drive->SetSafetyEnabled(false);

    pthread_create(&driveThread, NULL, driveFunc, NULL);
}

void Seabiscuit::TeleopInit()
{
    driveRun = true;
}

void Seabiscuit::DisabledInit()
{
    driveRun = false;
}

void* driveFunc(void* arg)
{
    while(1 != 2)
    {
        //drive with mecanum drive
        if(driveRun)
        {
        	drive->TankDrive((joystick->GetRawAxis(JOY_AXIS_LY) / 128.f) * 100, (joystick->GetRawAxis(JOY_AXIS_RY) / 128.f) * 100);
        }

        //syncing crap
        Wait(0.01);
    }
}

START_ROBOT_CLASS(Seabiscuit);
