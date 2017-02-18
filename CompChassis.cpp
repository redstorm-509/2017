#include <WPILib.h>
#include <CanTalonSRX.h>
#include <math.h>

/*
 * Get Distance function
 * Preferably calibrate wheel speeds for carpet
 */


class Robot: public frc::SampleRobot {



float shootercenterx = 153;
std::vector<double> visionX;
std::vector<double> visionY;

float Snotarget = .2;
float Sminimum = .05;
float desireddistance = 80;
float distmaxspeed = .6;
float distminspeed = .08;

float PI = 3.14159;

bool IsShooting = false;

int EncoderDelayLoop = 0;
float ShooterCurrentSpeed = 0;

int i2crainbow = 1;
int i2cred = 2;
int i2cblue = 3;
int i2cloading = 4;
int i2cshooting = 5;
int i2cendactions = 6;
int i2chastarget = 7;
int i2cnotarget = 8;

float gyroenabled = true;

public:
	Robot() {
		// hmmmmmmm... Nothing goes here now :/
	}

	void SetShooter(float DesiredEncoder) {
		SmartDashboard::PutBoolean("Wtf",true);
		if (EncoderDelayLoop == 0) {

			float CurrentEncoder = sampleEncoder->GetRate();
			SmartDashboard::PutNumber("CurrentE",CurrentEncoder);
			SmartDashboard::PutNumber("ShooterCurrentSpeed",ShooterCurrentSpeed);

			float DesiredSpeed;

			if ((ShooterCurrentSpeed==0) || (CurrentEncoder <= 0)) {
				DesiredSpeed = -.45;
			}
			else {
				DesiredSpeed = -((ShooterCurrentSpeed*DesiredEncoder)/(CurrentEncoder)); // fucking genius
			}

			if (DesiredSpeed>1) {
				DesiredSpeed = 1;
			}
			if (DesiredSpeed<-1) {
				DesiredSpeed = -1;
			}
			ShooterCurrentSpeed = DesiredSpeed;
			SmartDashboard::PutNumber("Shooter",DesiredSpeed);
			m_shooter.Set(DesiredSpeed);
		}
		EncoderDelayLoop += 1;
		if (EncoderDelayLoop >= 10) {
			EncoderDelayLoop = 0;
		}
	}

	void CAM() {
			visionX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
			visionY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
			if (visionX.size() > 0){
				ChangeColor(i2chastarget);
			}
			else {
				ChangeColor(i2cnotarget);
			}
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
			SmartDashboard::PutNumber("Y Center",yval);
			if (visionX.size() > 0) {
				SmartDashboard::PutNumber("X Center", visionX[0]);
			}
			float dist = (((0.47346)*(yval)) + (44.734));
			return dist;
		}
	}

	float BoilerSpin() {
		if ((visionX.size()) < 1) {
			return Snotarget;
		}
		else {
			float speed = ((((visionX[0] - shootercenterx)/shootercenterx)/(1/(Snotarget - Sminimum))));
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

	int GetDesiredEncoder() {
		float dist = GetDistance();
		//math
		int DesiredEncoder = (rstick.GetZ() * 10000);
		SmartDashboard::PutNumber("DesiredEncoder",DesiredEncoder);
		return DesiredEncoder;
	}

	void Accessories() {
		bool loadingpressed = opstick.GetRawButton(3);
		bool shootingpressed = (opstick.GetRawButton(5) && opstick.GetRawButton(6));
		if (loadingpressed){
			m_loader.Set(.95);
		}
		else{
			m_loader.Set(0);
		}
		int DesiredEncoderVal = 6000;
		if (visionX.size() > 0) {
			DesiredEncoderVal = GetDesiredEncoder();
		}

		if (shootingpressed){
			SmartDashboard::PutBoolean("Shooting",true);
			SetShooter(DesiredEncoderVal);
			if (walter.Get() == 0){
				walter.Start();
			}
			if (walter.Get() >= .1){
			servo.Set(.4);
			m_agitator.Set(.5);

			walter.Stop();
			IsShooting = true;
			}
		}

		else{
			SmartDashboard::PutBoolean("Shooting",false);
			EncoderDelayLoop = 0;
			m_agitator.Set(0);
			servo.Set(0);

			if (herbert.Get() == 0 && IsShooting == true){
				herbert.Start();
			}
			if ((herbert.Get() >= 2)){
			m_shooter.Set(0);
			IsShooting = false;

			herbert.Stop();
			herbert.Reset();

			}
			else if (IsShooting) {
				SetShooter(DesiredEncoderVal);
			}
			else {
				m_shooter.Set(0);
			}

			walter.Stop();
			walter.Reset();

		}

		///////////////////////////// I2C ////////////////////////////
		if (IsShooting) {
			ChangeColor(i2cshooting);
		}
		else if (loadingpressed) {
			ChangeColor(i2cloading);
		}
		else {
			if (opstick.GetRawButton(7)){
				ChangeColor(i2cred);
			}
			else if (opstick.GetRawButton(8)){
				ChangeColor(i2cblue);
			}
			else {
				ChangeColor(i2cendactions);
			}
		}
	}

	void ChangeColor(uint8_t val) {
			Wire.Transaction(&val,1,NULL,0);
	}


	void CanMechanum(float RX, float RY, float LX, float robotangle){
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
			RY = mag * cos(dif);
			RX = mag * sin(dif);
		}

		mag = sqrt((RY*RY)+(RX*RX));

		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX + RX;
		float rrVal = -RY + LX - RX;

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

		m_lf.Set(lfVal);
		m_lr.Set(lrVal);
		m_rf.Set(rfVal);
		m_rr.Set(rrVal);
	}

	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled()) {
			if (rstick.GetRawButton(3)) {
				gyro.Reset();
			}
			if (rstick.GetRawButton(8)) {
				gyro.Calibrate();
			}

			// Add in lstick buttons 2,3,4,5 to rotate "front"

			Accessories();

			// Make the next 2 statements work with a different "front"
			if (rstick.GetRawButton(4)) {
				m_rf.Set(-.7);
				m_lf.Set(-.7);
				m_rr.Set(.7);
				m_lr.Set(.7);
			}
			else if (rstick.GetRawButton(5)) {
				m_rf.Set(.7);
				m_lf.Set(.7);
				m_rr.Set(-.7);
				m_lr.Set(-.7);
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
		ChangeColor(i2cnotarget);
		ChangeColor(i2cendactions);
	}

private:

	// Drive Motors
	CanTalonSRX m_rf { 22 };
	CanTalonSRX m_rr { 23 };
	CanTalonSRX m_lf { 24 };
	CanTalonSRX m_lr { 20 };

	// Accessory Motors
	CanTalonSRX m_shooter { 27 };
	CanTalonSRX m_loader { 26 };
	CanTalonSRX m_agitator { 25 };
	frc::Servo servo { 0 };

	// Joysticks
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	frc::Joystick opstick { 2 };

	// Timers
	frc::Timer walter; // Lets shooting motor speed up
	frc::Timer herbert; // stops agitator before shooting motor

	// Gyro
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };

	// GRIP network table
	std::shared_ptr<NetworkTable> table { NetworkTable::GetTable("GRIP/BoilerReport1") };

	// Encoder
	Encoder *sampleEncoder = new Encoder(0,1, false, Encoder::EncodingType::k2X);

	//I2C initialization
	I2C Wire { I2C::Port::kOnboard,1 };


};

START_ROBOT_CLASS(Robot)
