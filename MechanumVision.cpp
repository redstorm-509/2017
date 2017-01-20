#include <Joystick.h>
#include <RobotDrive.h>
#include <SampleRobot.h>
#include <Timer.h>
#include <ADXRS450_Gyro.h>
#include <WPILib.h>

class Robot: public frc::SampleRobot {



float camcenterx = 153;
std::vector<double> visionX;
std::vector<double> visionY;

float Snotarget = .2;
float Sminimum = .05;
float desireddistance = 80;
float distmaxspeed = .3;
float distminspeed = .02;

public:
	Robot() {


		robotDrive.SetExpiration(0.1);

		// Invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);

		// You may need to change or remove this to match your robot
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}

	void CAM() {
				visionX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
				visionY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	}

	float ABS(float input) {
			if (input<0) {
				input = input * -1;
			}
			return input;
		}

	int GetMaxY() {
		int MaxY = -1;
		int MaxYIndex = -1;
		if (visionY.size() > 0) {
			for (unsigned int i=0; i < visionY.size(); i++) {
				if (visionY[i] > MaxY){
					MaxY = visionY[i];
					MaxYIndex = i;
				}
			}
		}
		return MaxYIndex;
	}

	float GetDistance() {
		int indexval = GetMaxY();
		if (indexval == -1) {
			return -1;
		}
		else {
			float yval = visionY[indexval];
			float dist = (((0.47346)*(yval)) + (44.734));
			return dist;
		}
	}

	float BoilerSpin() {
		if ((visionX.size()) < 1) {
			return Snotarget;
		}
		else {
			float speed = ((((visionX[0] - camcenterx)/camcenterx)/(1/(Snotarget - Sminimum))));
			if (speed<0) {
				speed -= Sminimum;
			}
			else {
				speed += Sminimum;
			}
			return speed;
		}
	}

	float AdjustDistance() {
		float dist = GetDistance();
		if (dist < 0) {
			return 0;
		}
		if (dist > (2 * desireddistance)) {
			dist = (2 * desireddistance);
		}
		float speed = ((((dist - desireddistance)/desireddistance)/(1/(distmaxspeed - distminspeed))));

		if (speed<0) {
			speed -= distminspeed;
		}
		else {
			speed += distminspeed;
		}
		SmartDashboard::PutNumber("Speed",speed);
		return speed;
	}

	void OperatorControl() override {
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled()) {
			if (rstick.GetRawButton(3)) {
				gyro.Reset();
			}

			if (not(rstick.GetRawButton(1))) {
				robotDrive.MecanumDrive_Cartesian(rstick.GetX(), rstick.GetY(), lstick.GetX(), gyro.GetAngle());
			}
			else {
				if (lstick.GetRawButton(1)) {
					robotDrive.MecanumDrive_Cartesian(0, -(AdjustDistance()), BoilerSpin());
				}
				else {
					robotDrive.MecanumDrive_Cartesian(rstick.GetX(), rstick.GetY(), BoilerSpin(), gyro.GetAngle());
				}
			}

			CAM();

			frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

private:
	// Channels for the wheels
	static constexpr int kFrontLeftChannel = 2;
	static constexpr int kRearLeftChannel = 3;
	static constexpr int kFrontRightChannel = 1;
	static constexpr int kRearRightChannel = 0;

	// Robot drive system
	frc::RobotDrive robotDrive { kFrontLeftChannel, kRearLeftChannel,
			kFrontRightChannel, kRearRightChannel };
	// Only joystick
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };
	std::shared_ptr<NetworkTable> table { NetworkTable::GetTable("GRIP/BoilerReport1") };

};

START_ROBOT_CLASS(Robot)
