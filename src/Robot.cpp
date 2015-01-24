#include "WPILib.h"
#include "Robot.h"
#include "config.h"

void Seabiscuit::RobotInit() {
	drive = new RobotDrive(FRONT_LEFT_MOTOR_PWM, REAR_LEFT_MOTOR_PWM,
						   FRONT_RIGHT_MOTOR_PWM, REAR_RIGHT_MOTOR_PWM);
	joystick = new Joystick(JOY_PORT_0);
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
	bool precisionMode = false;
	float precisionFactor = 0.75f;

	while (true) {
		if (driveRun) {
			precisionMode = joystick->GetRawButton(JOY_BTN_RBM);

			float xSpeed = joystick->GetRawAxis(JOY_AXIS_LX);
			float zSpeed = -joystick->GetRawAxis(JOY_AXIS_LY);
			float ySpeed = -joystick->GetRawAxis(JOY_AXIS_RX);

			if (precisionMode) {
				xSpeed *= precisionFactor;
				ySpeed *= precisionFactor;
				zSpeed *= precisionFactor;
			}
			drive->MecanumDrive_Cartesian(xSpeed,
										  ySpeed,
										  zSpeed);
		}
	}
}

START_ROBOT_CLASS(Seabiscuit);
