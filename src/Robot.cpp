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
	joystick = new Joystick(JOY_PORT_0);
	gyro = new Gyro(GYRO_CHANNEL);
	gyro->InitGyro();

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
	float Kp = 0.024000;//I
	float Ki = 0.021000;//Love
	float Kd = 0.000420;//Integrals
	SmartDashboard::PutString("DB/String 0", std::to_string(Kp));
	SmartDashboard::PutString("DB/String 1", std::to_string(Ki));
	SmartDashboard::PutString("DB/String 2", std::to_string(Kd));

	float lPIDError = 0;
	float lPIDIntegral = 0;
	float lCurrentSpeed = 0; //lel
	float rPIDError = 0;
	float rPIDIntegral = 0;
	float rCurrentSpeed = 0; //lel

	bool precisionMode = false;

	float joyDifference = joystick->GetRawAxis(JOY_AXIS_LY) - joystick->GetRawAxis(JOY_AXIS_RY);

	double oldtime = GetTime();
	while (true) {
		//drive with tank drive for now
		double ctime = GetTime();
		if (driveRun) {
			Kp = std::stof(SmartDashboard::GetString("DB/String 0"));
			Ki = std::stof(SmartDashboard::GetString("DB/String 1"));
			Kd = std::stof(SmartDashboard::GetString("DB/String 2"));
			//saving K values in dashboard q:^ )

			precisionMode = joystick->GetRawButton(JOY_BTN_RBM);
			if (precisionMode) {
				//teehee
				Kp /= 2;
				Ki /= 2;
				Kd /= 2;
			}

			/*joyDifference = joystick->GetRawAxis(JOY_AXIS_LY) - joystick->GetRawAxis(JOY_AXIS_RY);
			if (!(joyDifference >= -0.07 && joyDifference <= 0.07)) {
				if (fabs(gyro->GetAngle()) > 0.2)
					gyro->Reset();
				SmartDashboard::PutString("DB/String 3", "Waiting for alignment...");
			}
			else {
				SmartDashboard::PutString("DB/String 3", std::to_string(gyro->GetAngle()));
			}*/

			//Left PID loop
			float lCurrentError = joystick->GetRawAxis(JOY_AXIS_LY) - lCurrentSpeed;
			lPIDIntegral += lPIDError * (ctime - oldtime);
			float lPIDderivative = (lCurrentError - lPIDError) / (ctime - oldtime);
			lCurrentSpeed += (Kp * lCurrentError) + (Ki * lPIDIntegral) + (Kd * lPIDderivative);
			lPIDError = lCurrentError;

			//Right PID loop
			float rCurrentError = joystick->GetRawAxis(JOY_AXIS_RY) - rCurrentSpeed;
			rPIDIntegral += rPIDError * (ctime - oldtime);
			float rPIDderivative = (rCurrentError - rPIDError) / (ctime - oldtime);
			rCurrentSpeed += (Kp * rCurrentError) + (Ki * rPIDIntegral) + (Kd * rPIDderivative);
			rPIDError = rCurrentError;

			SmartDashboard::PutString("DB/String 5", std::to_string(lCurrentSpeed));
			SmartDashboard::PutString("DB/String 6", std::to_string(rCurrentSpeed));
			lCurrentSpeed = fabs(lCurrentSpeed) > 1 ? sgn(lCurrentSpeed) : lCurrentSpeed;
			rCurrentSpeed = fabs(rCurrentSpeed) > 1 ? sgn(rCurrentSpeed) : rCurrentSpeed;
			SmartDashboard::PutString("DB/String 7", std::to_string(lCurrentSpeed));
			SmartDashboard::PutString("DB/String 8", std::to_string(rCurrentSpeed));

			if (precisionMode) {
				drive->TankDrive((lCurrentSpeed/* - sgn(lCurrentSpeed) * (gyro->GetAngle() / 360.f)*/) / 1.5f, (rCurrentSpeed/* + sgn(rCurrentSpeed) * (gyro->GetAngle() / 360.f)*/) / 1.5f);
			}
			else {
				drive->TankDrive(lCurrentSpeed/* - sgn(lCurrentSpeed) * (gyro->GetAngle() / 360.f)*/, rCurrentSpeed/* + sgn(rCurrentSpeed) * (gyro->GetAngle() / 360.f)*/);
			}
		}
		//drive->TankDrive(joystick->GetRawAxis(JOY_AXIS_LY),joystick->GetRawAxis(JOY_AXIS_RY));
		oldtime = ctime;
	}
}

START_ROBOT_CLASS(Seabiscuit);
