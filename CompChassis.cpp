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

class Robot: public frc::SampleRobot {



float camcenterx = 153;
std::vector<double> visionX;
std::vector<double> visionY;

float Snotarget = .2;
float Sminimum = .05;
float desireddistance = 80;
float distmaxspeed = .3;
float distminspeed = .02;
float PI = 3.14159;

int i2cwhite = 0;
int i2crainbow = 1;
int i2credteam = 2;
int i2cblueteam = 3;
int i2cloading = 4;
int i2cshooting = 5;
int i2cendactions = 6;

float gyroenabled = false;

public:
	Robot() {


		//robotDrive.SetExpiration(0.1);

		// Invert the left side motors
		//robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);

		// You may need to change or remove this to match your robot
	//	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}

	/*void CAM() {
				visionX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
				visionY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	}*/

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
	
	void ChangeColor(uint8_t val) {
			Wire.Transaction(&val,1,NULL,0);
	}


	void CanMechanum(float RX, float RY, float LX, float robotangle){
		RX = -RX;
		float mag = sqrt((RY*RY)+(RX*RX));
		if (gyroenabled) {
			//robotangle = ((360/4.8)*(-(robotangle)));
			robotangle = (((PI)/180)*robotangle);
			SmartDashboard::PutNumber("Initial Gyro",(robotangle/PI));
			SmartDashboard::PutNumber("Initial RY",RY);
			SmartDashboard::PutNumber("Initial RX",RX);
			//SmartDashboard::PutNumber("Modified Gyro",robotangle);
			while ((robotangle >= (2*PI)) or (robotangle < 0)){
				if (robotangle >= (2*PI)){
					robotangle = robotangle - (2*PI);
				}

				else if (robotangle < 0){
					robotangle = robotangle + (2*PI);
				}
			}
			SmartDashboard::PutNumber("GyroAngle",robotangle);
			float joyangle = 0;
			float dif = 0;
			SmartDashboard::PutNumber("Magnitude",mag);
			//angle = angle * (PI/180);
			if ((RX>=0) && (RY>0)) { // Top Right
				SmartDashboard::PutBoolean("TopRightQuad",true);
				SmartDashboard::PutBoolean("BottomRightQuad",false);
				SmartDashboard::PutBoolean("BottomLeftQuad",false);
				SmartDashboard::PutBoolean("TopLeftQuad",false);
				joyangle = 0;
				//if (not(RX==0)) {
					joyangle = atan(RX/RY) + joyangle;
				//}
				/*else {
					//joyangle = ((180/PI)*joyangle);
				}*/
				dif = joyangle - robotangle;
				//dif = -dif;
				RY = mag*cos(dif);
				RX = mag*sin(dif);
			}
			else if ((RX>0) && (RY<=0)) { // Bottom Right
				SmartDashboard::PutBoolean("TopRightQuad",false);
				SmartDashboard::PutBoolean("BottomRightQuad",true);
				SmartDashboard::PutBoolean("BottomLeftQuad",false);
				SmartDashboard::PutBoolean("TopLeftQuad",false);
				joyangle = ((PI)/2);
				//if (not(RY==0)) {
					//joyangle = 90;
					//joyangle = 0;
					joyangle = (-atan(RY/RX)) + joyangle;
				//}
				/*else {
					//joyangle = -((180/PI)*joyangle);
				}*/
				dif = joyangle - robotangle;
				//dif = -dif;
				RX = mag*sin(dif);
				RY = mag*cos(dif);
			}
			else if ((RX<=0) && (RY<0)) { // Bottom Left
				SmartDashboard::PutBoolean("TopRightQuad",false);
				SmartDashboard::PutBoolean("BottomRightQuad",false);
				SmartDashboard::PutBoolean("BottomLeftQuad",true);
				SmartDashboard::PutBoolean("TopLeftQuad",false);
				joyangle = (PI);
				//if (not(RX==0)) {
					joyangle = atan(RX/RY) + joyangle;
					//joyangle = 180;
					//joyangle = 0;
				//}
				/*else {

					//joyangle = ((180/PI)*joyangle);
				}*/
				dif = joyangle - robotangle;
				//dif = -dif;
				RY = mag*cos(dif);
				RX = mag*sin(dif);
			}
			else if ((RX<0) && (RY>=0)) { // Top Left
				SmartDashboard::PutBoolean("TopRightQuad",false);
				SmartDashboard::PutBoolean("BottomRightQuad",false);
				SmartDashboard::PutBoolean("BottomLeftQuad",false);
				SmartDashboard::PutBoolean("TopLeftQuad",true);
				joyangle = ((3*PI)/2);
				//if (not(RY==0)) {
					//joyangle = 270;
					//joyangle = 0;
					joyangle = atan(RY/RX) + joyangle;
				//}
				/*else {
					//joyangle = -((180/PI)*joyangle);
				}*/
				dif = joyangle - robotangle;
				//dif = -dif;
				RX = mag*sin(dif);
				RY = -mag*cos(dif);
			}
			SmartDashboard::PutNumber("joyangle",(joyangle/PI));
			SmartDashboard::PutNumber("Dif",(dif/PI));
			SmartDashboard::PutNumber("Final RY",RY);
			SmartDashboard::PutNumber("Final RX",RX);
		}



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

		/*std::vector<float> Values;
		float MaxVal = -1;
		int indexVal = 0;
		for (unsigned int i=0; i < Values.size(); i++) {
			if (Values[i] > MaxVal){
				MaxVal = Values[i];
				indexVal = i;
			}
		}*/

		if (ABS(maxval) > 1){
			lfVal = ((lfVal / ABS(maxval))*(mag/sqrt(2)));
			lrVal = ((lrVal / ABS(maxval))*mag/sqrt(2));
			rfVal = ((rfVal / ABS(maxval))*mag/sqrt(2));
			rrVal = ((rrVal / ABS(maxval))*mag/sqrt(2));
		}

		SmartDashboard::PutNumber("Left Front",lfVal);
		SmartDashboard::PutNumber("Left Rear",lrVal);
		SmartDashboard::PutNumber("Right Front",rfVal);
		SmartDashboard::PutNumber("Right Rear",rrVal);

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
			/*if (rstick.GetRawButton(4)) {
				lfm.Set(-.5);
				lrm.Set(.5);
				rfm.Set(-.5);
				rrm.Set(.5);
			}
			else if (rstick.GetRawButton(5)) {
				lfm.Set(.5);
				lrm.Set(-.5);
				rfm.Set(.5);
				rrm.Set(-.5);
			}
			else {
				lfm.Set(0);
				lrm.Set(0);
				rrm.Set(0);
				rfm.Set(0);
			}*/

			CanMechanum(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), gyro.GetAngle());

			//if (not(rstick.GetRawButton(1))) {
				//CanMechanum();

				//robotDrive.MecanumDrive_Cartesian(rstick.GetX(), rstick.GetY(), lstick.GetX(), gyro.GetAngle());
			//}
			/*else {
				if (lstick.GetRawButton(1)) {
					robotDrive.MecanumDrive_Cartesian(0, -(AdjustDistance()), BoilerSpin());
				}
				else {
					robotDrive.MecanumDrive_Cartesian(rstick.GetX(), rstick.GetY(), BoilerSpin(), gyro.GetAngle());
				}
			}*/

			//CAM();

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

	// Robot drive system

	//CANTalon::CANTalon rfm { 22 };

	CanTalonSRX rfm { kFrontRightChannel };
	CanTalonSRX rrm { kRearRightChannel };
	CanTalonSRX lfm { kFrontLeftChannel };
	CanTalonSRX lrm { kRearLeftChannel };

	// TRUMAN NOTE THAT "CANTalon.h" with "CANTalon" instead of "CanTalonSRX" in the lines above does
	// not error, however it doesn't run the motors. Just a note

	//frc::RobotDrive robotDrive { lfm, lrm,
	//	rfm, rrm };

	/*frc::RobotDrive robotDrive { CanTalonSRX::CanTalonSRX {kFrontLeftChannel}, CanTalonSRX::CanTalonSRX {kRearLeftChannel},
		CanTalonSRX::CanTalonSRX {kFrontRightChannel}, CanTalonSRX::CanTalonSRX {kRearRightChannel}};
*/
	// Only joystick
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };
	//std::shared_ptr<NetworkTable> table { NetworkTable::GetTable("GRIP/BoilerReport1") };
	
	//I2C initilazation
	I2C Wire { I2C::Port::kOnboard,1 };


};

START_ROBOT_CLASS(Robot)
