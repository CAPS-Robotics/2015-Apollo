#include "WPILib.h"
#include "Robot.h"
#include "config.h"

Seabiscuit::~Seabiscuit() {
	delete drive;
	delete joystick;
	delete gyro;
}

void Seabiscuit::RobotInit() {
	drive = new RobotDrive(LEFT_MOTOR_PWM, RIGHT_MOTOR_PWM);
	joystick = new Joystick(JOY_PORT_1);
	gyro = new Gyro(GYRO_CHANNEL);

	drive->SetSafetyEnabled(false);

	pthread_create(&driveThread, NULL, driveFunc, NULL);
}

void Seabiscuit::TeleopInit() {
	driveRun = true;
}

void Seabiscuit::DisabledInit() {
	driveRun = false;
}

void* driveFunc(void* arg) {
	float stickDif;
	float stickAvg;
	float gyroAngle;
	while (1 != 2) {
		//drive with tank drive for now
		if (driveRun) {
			stickDif = joystick->GetRawAxis(JOY_AXIS_LY) - joystick->GetRawAxis(JOY_AXIS_RY);
			gyro->Reset();
			//gyro based drive for straightness if sticks have difference of +- 3
			while (stickDif <= 3 && stickDif >= -3) {
				gyroAngle = gyro->GetAngle();
				stickAvg = (joystick->GetRawAxis(JOY_AXIS_LY) + joystick->GetRawAxis(JOY_AXIS_RY)) / 2.f;
				drive->TankDrive(((stickAvg / 128.f) * 100) - gyroAngle, ((stickAvg / 128.f) * 100) + gyroAngle);
				stickDif = joystick->GetRawAxis(JOY_AXIS_LY) - joystick->GetRawAxis(JOY_AXIS_RY);
			}
			drive->TankDrive((joystick->GetRawAxis(JOY_AXIS_LY) / 128.f) * 100, (joystick->GetRawAxis(JOY_AXIS_RY) / 128.f) * 100);
		}

		//syncing crap
		Wait(0.01);
	}
}

START_ROBOT_CLASS(Seabiscuit);
