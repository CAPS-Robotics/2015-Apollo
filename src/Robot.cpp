#include "WPILib.h"
#include "Robot.h"
#include "config.h"

Seabiscuit::~Seabiscuit() {
	delete drive;
	delete joystick;
}

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
	float Kp = 0.024000; //A
	float Ki = 0.021000; //O
	float Kd = 0.000420; //L
	//mail
	SmartDashboard::PutString("DB/String 0", std::to_string(Kp));
	SmartDashboard::PutString("DB/String 1", std::to_string(Ki));
	SmartDashboard::PutString("DB/String 2", std::to_string(Kd));

	float xPIDError = 0;
	float xPIDIntegral = 0;
	float xCurrentSpeed = 0; //top
	float yPIDError = 0;
	float yPIDIntegral = 0;
	float yCurrentSpeed = 0; //lel
	float zPIDError = 0;
	float zPIDIntegral = 0;
	float zCurrentSpeed = 0; //lel

	bool precisionMode = false;
	float precisionFactor = 0.75f;

	double oldtime = GetTime();
	while (true) {
		double ctime = GetTime();
		if (driveRun) {
			Kp = std::stof(SmartDashboard::GetString("DB/String 0"));
			Ki = std::stof(SmartDashboard::GetString("DB/String 1"));
			Kd = std::stof(SmartDashboard::GetString("DB/String 2"));
			//saving K values in dashboard q:^ )

			precisionMode = joystick->GetRawButton(JOY_BTN_RBM);

			//Left PID loop
			float xCurrentError = joystick->GetRawAxis(JOY_AXIS_LX) - xCurrentSpeed;
			xPIDIntegral += xPIDError * (ctime - oldtime);
			float xPIDderivative = (xCurrentError - xPIDError) / (ctime - oldtime);
			xCurrentSpeed += (Kp * xCurrentError) + (Ki * xPIDIntegral) + (Kd * xPIDderivative);
			xPIDError = xCurrentError;

			//Right PID loop
			float yCurrentError = joystick->GetRawAxis(JOY_AXIS_LY) - yCurrentSpeed;
			yPIDIntegral += yPIDError * (ctime - oldtime);
			float yPIDderivative = (yCurrentError - yPIDError) / (ctime - oldtime);
			yCurrentSpeed += (Kp * yCurrentError) + (Ki * yPIDIntegral) + (Kd * yPIDderivative);
			yPIDError = yCurrentError;

			//Rotate PID loop
			float zCurrentError = joystick->GetRawAxis(JOY_AXIS_RX) - zCurrentSpeed;
			zPIDIntegral += zPIDError * (ctime - oldtime);
			float zPIDderivative = (zCurrentError - zPIDError) / (ctime - oldtime);
			zCurrentSpeed += (Kp * zCurrentError) + (Ki * zPIDIntegral) + (Kd * zPIDderivative);
			zPIDError = zCurrentError;

			xCurrentSpeed = safe_motor(xCurrentSpeed);
			yCurrentSpeed = safe_motor(yCurrentSpeed);
			zCurrentSpeed = safe_motor(zCurrentSpeed);
			SmartDashboard::PutString("DB/String 5", std::to_string(xCurrentSpeed));
			SmartDashboard::PutString("DB/String 6", std::to_string(yCurrentSpeed));
			SmartDashboard::PutString("DB/String 7", std::to_string(zCurrentSpeed));

			if (precisionMode) {
				drive->MecanumDrive_Cartesian(xCurrentSpeed * precisionFactor,
								 	yCurrentSpeed * precisionFactor,
									zCurrentSpeed * precisionFactor);
			} else {
				drive->MecanumDrive_Cartesian(xCurrentSpeed,
								 	yCurrentSpeed,
									zCurrentSpeed);
			}
		}
		oldtime = ctime;
	}
}

START_ROBOT_CLASS(Seabiscuit);
