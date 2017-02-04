#include <Joystick.h>
#include <RobotDrive.h>
#include <SampleRobot.h>
#include <Timer.h>
#include <ADXRS450_Gyro.h>
#include <WPILib.h>
#include <CanTalonSRX.h>
#include <math.h>
#include <WPILib.h>
#include <I2C.h>
#include <Servo.h>

class Robot: public frc::SampleRobot {



float camcenterx = 153;
std::vector<double> visionX;
std::vector<double> visionY;

float Snotarget = .2;
float Sminimum = .05;
float desireddistance = 80;
float distmaxspeed = .6;
float distminspeed = .15;
float PI = 3.14159;

bool shootingStoped = false;

int i2cwhite = 0;
int i2crainbow = 1;
int i2credteam = 2;
int i2cblueteam = 3;
int i2cloading = 4;
int i2cshooting = 5;
int i2cendactions = 6;

float gyroenabled = true;

public:
	Robot() {


		//robotDrive.SetExpiration(0.1);

		// Invert the left side motors
		//robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);

		// You may need to change or remove this to match your robot
	//	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
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

	void Accessories() {
		bool loadingpressed = opstick.GetRawButton(3);
		bool shootingpressed = (opstick.GetRawButton(5) && opstick.GetRawButton(6));
		if (loadingpressed){
			loader.Set(.95);
		}
		else{
			loader.Set(0);
		}

		if (shootingpressed){
			if (walter.Get() == 0){
				walter.Start();

				shooter.Set(.95);
			}
			if (walter.Get() >= .1){
			servo.Set(.4);
			agitator.Set(.5);

			walter.Stop();
			shootingStoped = true;
			}

		}

		else{


			agitator.Set(0);
			servo.Set(0);

			if (herbert.Get() == 0 && shootingStoped == true){
				herbert.Start();

				shootingStoped = false;
			}
			if (herbert.Get() >= 2){
			shooter.Set(0);

			herbert.Stop();
			herbert.Reset();
			}

			walter.Stop();
			walter.Reset();

		}

		///////////////////////////// I2C ////////////////////////////
		if (shootingpressed) {
			ChangeColor(i2cshooting);
		}
		else if (loadingpressed) {
			ChangeColor(i2cloading);
		}
		else {
			ChangeColor(i2cendactions);
		}
	}

	void ChangeColor(uint8_t val) {
			Wire.Transaction(&val,1,NULL,0);
	}


	void CanMechanum(float RX, float RY, float LX, float robotangle){
		//RX = -RX;
		float mag = sqrt((RY*RY)+(RX*RX));
		float joyangle = 0;
		if (gyroenabled) {
			robotangle = (((PI)/180)*robotangle);
			while ((robotangle >= (2*PI)) or (robotangle < 0)){
				if (robotangle >= (2*PI)){
					robotangle = robotangle - (2*PI);
				}

				else if (robotangle < 0){
					robotangle = robotangle + (2*PI);
				}
			}
			SmartDashboard::PutNumber("Gyro",robotangle*(180/PI));
			float dif = 0;
			if ((RX>=0) && (RY>0)) { // Top Right
				joyangle = atan(RX/RY) + 0;
				dif = joyangle - robotangle;
			}
			else if ((RX>0) && (RY<=0)) { // Bottom Right
				joyangle = (atan((-RY)/RX)) + ((PI)/2);
				dif = joyangle - robotangle;
			}
			else if ((RX<=0) && (RY<0)) { // Bottom Left
				joyangle = atan((-RX)/(-RY)) + (PI);
				dif = joyangle - robotangle;
			}
			else if ((RX<0) && (RY>=0)) { // Top Left
				joyangle = atan(RY/(-RX)) + ((3*PI)/2);
				dif = joyangle - robotangle;
			}
			SmartDashboard::PutNumber("Dif",dif*(180/PI));
			RY = mag * cos(dif);
			RX = mag * sin(dif);
		}

		SmartDashboard::PutNumber("JoyAngle",joyangle*(180/PI));

		mag = sqrt((RY*RY)+(RX*RX));

		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX - RX;
		float rrVal = -RY + LX + RX;

		float maxval = lfVal;
		if (lrVal>maxval) {
			maxval = lrVal;
		}
		if (rfVal>maxval) {
			maxval = rfVal;
		}
		if (rrVal>maxval) {
			maxval = rrVal;
		}

		if (ABS(maxval) > 1){
			lfVal = ((lfVal / ABS(maxval))*(mag/sqrt(2)));
			lrVal = ((lrVal / ABS(maxval))*mag/sqrt(2));
			rfVal = ((rfVal / ABS(maxval))*mag/sqrt(2));
			rrVal = ((rrVal / ABS(maxval))*mag/sqrt(2));
		}

		lfm.Set(lfVal);
		lrm.Set(lrVal);
		rfm.Set(rfVal);
		rrm.Set(rrVal);
	}

	void OperatorControl() override {
		//robotDrive.SetSafetyEnabled(false);
		//gyro.Reset();

		ChangeColor(i2credteam);

		while (IsOperatorControl() && IsEnabled()) {

			if (rstick.GetRawButton(3)) {
				gyro.Reset();
			}
			if (rstick.GetRawButton(2)) {
				gyro.Calibrate();
			}

			Accessories();

		//	CanMechanum(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), gyro.GetAngle());

			if (rstick.GetRawButton(4)) {
				rfm.Set(-.7);
				lfm.Set(-.7);
				rrm.Set(.7);
				lrm.Set(.7);
			}
			else if (rstick.GetRawButton(5)) {
				rfm.Set(.7);
				lfm.Set(.7);
				rrm.Set(-.7);
				lrm.Set(-.7);
			}
			else {
				if (not(rstick.GetRawButton(1))) {
					CanMechanum(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), gyro.GetAngle());
				}
				else {
					if (lstick.GetRawButton(1)) {
						CanMechanum((AdjustDistance()), 0, BoilerSpin(), 0);
					}
					else {
						CanMechanum(rstick.GetX(), -rstick.GetY(), BoilerSpin(), gyro.GetAngle());
					}
				}
			}


			CAM();

			frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
		ChangeColor(i2crainbow);
	}

private:
	// Channels for the wheels
	static constexpr int kFrontLeftChannel = 23;
	static constexpr int kRearLeftChannel = 22;
	static constexpr int kFrontRightChannel = 20;
	static constexpr int kRearRightChannel = 24;
	static constexpr int kShooterChannel = 25;
	static constexpr int kloadingChannel = 26;
	static constexpr int kAgitatorChannel = 27;

	// Robot drive system

	//CANTalon::CANTalon rfm { 22 };

	/*CanTalonSRX rfm { kFrontRightChannel };
	CanTalonSRX rrm { kRearRightChannel };
	CanTalonSRX lfm { kFrontLeftChannel };
	CanTalonSRX lrm { kRearLeftChannel };*/
	CanTalonSRX shooter { kShooterChannel };
	CanTalonSRX loader { kloadingChannel };
	CanTalonSRX agitator { kAgitatorChannel };

	frc::Talon lfm { 0 };
	frc::Talon lrm { 1 };
	frc::Talon rfm { 2 };
	frc::Talon rrm { 3 };

	frc::Servo servo { 0 };


	//frc::RobotDrive robotDrive { lfm, lrm,
	//	rfm, rrm };

	/*frc::RobotDrive robotDrive { CanTalonSRX::CanTalonSRX {kFrontLeftChannel}, CanTalonSRX::CanTalonSRX {kRearLeftChannel},
		CanTalonSRX::CanTalonSRX {kFrontRightChannel}, CanTalonSRX::CanTalonSRX {kRearRightChannel}};
*/
	// Only joystick
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	frc::Joystick opstick { 2 };
	frc::Timer walter;
	frc::Timer herbert;
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };
	std::shared_ptr<NetworkTable> table { NetworkTable::GetTable("GRIP/BoilerReport1") };

	//I2C initilazation
	I2C Wire { I2C::Port::kOnboard,1 };


};

START_ROBOT_CLASS(Robot)
