#include <WPILib.h>
#include <CanTalonSRX.h>
#include <math.h>

/*
 * DESIRED ENCODER VALUE IS 8976
 * DESIRED Y VALUE IS 207
 * FIND DESIRED X VALUE = 496
 * AdjustToY() is untested, but should work?
 *
 * FYI I changed GetDistance() so it doesn't actually return a distance...
 * oops
 */


class Robot: public frc::SampleRobot {

float shootercenterx = 496;
std::vector<double> visionX;
std::vector<double> visionY;

float Snotarget = .3;
float Sminimum = .15;
float desireddistance = 124.5;
float distmaxspeed = .6;
float distminspeed = .12;
float TurnMax = .5;
float TurnMin = .27;

int desiredYVal = 207;

float PI = 3.14159;

bool IsShooting = false;
//bool autoComplete = false;

bool ManualShooter = false;

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
		if (EncoderDelayLoop == 0) {

			float CurrentEncoder = sampleEncoder->GetRate();
			SmartDashboard::PutNumber("Current Encoder", CurrentEncoder);

			float DesiredSpeed = rstick.GetZ();

			if (not(ManualShooter)) {
				if ((ShooterCurrentSpeed==0) || (CurrentEncoder <= 0)) {
					DesiredSpeed = -.45;
				}
				else {
					DesiredSpeed = ((ShooterCurrentSpeed*DesiredEncoder)/(CurrentEncoder)); // fucking genius
				}

				if (DesiredSpeed>1) {
					DesiredSpeed = 1;
				}
				if (DesiredSpeed<-1) {
					DesiredSpeed = -1;
				}
				ShooterCurrentSpeed = DesiredSpeed;
			}
			if (DesiredSpeed!=-.45) {
				SmartDashboard::PutNumber("Speed",DesiredSpeed);
			}
			m_shooter.Set(DesiredSpeed);
			//m_shooter.Set(rstick.GetZ());
		}
		EncoderDelayLoop += 1;
		if (EncoderDelayLoop >= 15) {
			EncoderDelayLoop = 0;
		}
	}

	void CAM() {
			visionX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
			if (visionX.size()>0) {
				SmartDashboard::PutNumber("VisionX",visionX[0]);
			}
			visionY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
			if (visionY.size()>0) {
				SmartDashboard::PutNumber("VisionY",visionY[0]);
			}
			/*if (visionX.size() > 0){
				ChangeColor(i2chastarget);
			}
			else {
				ChangeColor(i2cnotarget);
			}*/
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
			if (visionX.size() > 0) {
				//SmartDashboard::PutNumber("X Center", visionX[0]);
			}
			//float dist = (((0.47346)*(yval)) + (44.734));
			//return dist;
			return yval;
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

	float AdjustToY() {
		float yval = GetDistance();
		if (yval < 0) {
			return 0;
		}
		if (yval > 2 * desiredYVal) {
			yval = 2 * desiredYVal;
		}
		float speed = ((((yval - desiredYVal)/desiredYVal)/(1/(distmaxspeed - distminspeed))));

		if (speed<0) {
			speed -= distminspeed;
		}
		else {
			speed += distminspeed;
		}
		return speed;

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
		return speed;
	}

	int GetDesiredEncoder() {
		float dist = GetDistance();
		//math
		//int DesiredEncoder = (rstick.GetZ() * 10000);
		int DesiredEncoder = (rstick.GetZ() * 10000);
		SmartDashboard::PutNumber("Current Ed",rstick.GetZ() * 10000);
		DesiredEncoder = 8976;
		return DesiredEncoder;
	}

	void Accessories() {
		//bool mouseTrap = false;
		bool loadingpressed = opstick.GetRawButton(3);
		bool shootingpressed = (opstick.GetRawButton(5) && opstick.GetRawButton(6));
/*
		if (fatherTime.Get() >= 105 && (opstick.GetThrottle() >= .4 || mouseTrap) && autoComplete){
			m_climber.Set(-opstick.GetThrottle());
			mouseTrap = true;
		}
		else if ((opstick.GetThrottle() >= .4 || mouseTrap) && !autoComplete){
			m_climber.Set(-opstick.GetThrottle());
			mouseTrap = true;
		}
		else {
			m_climber.Set(0);
		}*/
		if (not(opstick.GetRawButton(1))){
			m_climber.Set(-opstick.GetThrottle());
		}
		else {
			m_climber.Set(-opstick.GetThrottle()/2);
		}

		if (loadingpressed){
			m_loader.Set(-.95);
		}
		else{
			m_loader.Set(0);
		}
		int DesiredEncoderVal = GetDesiredEncoder();//6000;
		if (visionX.size() > 0) {
			DesiredEncoderVal = GetDesiredEncoder();
		}

		if (shootingpressed){
			SetShooter(DesiredEncoderVal);
			if (walter.Get() == 0){
				walter.Start();
			}
			if (walter.Get() >= .5){
				hopperstop.Set(1);
				m_agitator.Set(-.7);

				walter.Stop();
				IsShooting = true;
			}
			else if (walter.Get() >= .4){
				hopperstop.Set(0);
			}
		}

		else{
			EncoderDelayLoop = 0;
			m_agitator.Set(0);
			hopperstop.Set(.5);

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

	void tanks4nothing(float RY, float LY){
		m_lf.Set(-LY);
		m_lr.Set(RY);
		m_rf.Set(-LY);
		m_rr.Set(RY);
	}

	void OperatorControl() override {
		fatherTime.Reset();
		fatherTime.Start();
		loaderstop.Set(.56);

		while (IsOperatorControl() && IsEnabled()) {
			comp->SetClosedLoopControl(true);
			if (rstick.GetRawButton(3)) {
				gyro.Reset();
			}
			if (rstick.GetRawButton(8)) {
				gyro.Calibrate();
			}

			// Add in lstick buttons 2,3,4,5 to rotate "front"

			Accessories();
			if (opstick.GetRawButton(1)) {
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
			} else if (not(opstick.GetRawButton(1))) {
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
			} else {
				m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
			}

			// Make the next 2 statements work with a different "front"
			if (rstick.GetRawButton(4)) {
				m_rf.Set(-.85);
				m_lf.Set(.85);
				m_rr.Set(-.85);
				m_lr.Set(.85);
			}
			else if (rstick.GetRawButton(5)) {
				m_rf.Set(.85);
				m_lf.Set(-.85);
				m_rr.Set(.85);
				m_lr.Set(-.85);
			}
			else if (lstick.GetRawButton(4)) {
				m_rf.Set(-.55);
				m_lf.Set(.55);
				m_rr.Set(-.55);
				m_lr.Set(.55);
			}
			else if (lstick.GetRawButton(5)) {
				m_rf.Set(.44);
				m_lf.Set(-.55);
				m_rr.Set(.55);
				m_lr.Set(-.55);
			}
			else {
				if (lstick.GetRawButton(2)) { // toggle climber turning
					if (lstick.GetRawButton(9)) { // climber to center
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo(0-90),gyro.GetAngle());
					}
					else if (lstick.GetRawButton(10)) { // climber to left
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo(30-90),gyro.GetAngle());
					}
					else if (lstick.GetRawButton(11)) { // climber to right
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo((-30)-90),gyro.GetAngle());
					}
					else { // not force spinning
						CanMechanum(rstick.GetX(),-rstick.GetY(),(lstick.GetX()/2),gyro.GetAngle());
					}
				}
				else {
					if (lstick.GetRawButton(9)) { // gear to center
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo(0),gyro.GetAngle());
					}
					else if (lstick.GetRawButton(10)) { // gear to left
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo(30),gyro.GetAngle());
					}
					else if (lstick.GetRawButton(11)) { // gear to right
						CanMechanum(rstick.GetX(),-rstick.GetY(),TurnTo(-30),gyro.GetAngle());
					}
					else if (not(rstick.GetRawButton(1))) {
						CanMechanum(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), gyro.GetAngle());
						//tanks4nothing(rstick.GetY(), lstick.GetY());
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
			}


			CAM();

			frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
		//ChangeColor(i2cnotarget);
		ChangeColor(i2cendactions);
	}

	void SetAll(float val) {
		m_rf.Set(val);
		m_rr.Set(val);
		m_lf.Set(val);
		m_lr.Set(val);
	}

	float TurnTo(int theta) {
			while (theta >= 360) {
				theta -= 360;
			}
			while (theta < 0) {
				theta += 360;
			}
			float robotangle = gyro.GetAngle();
			while (robotangle >= 360) {
				robotangle -= 360;
			}
			while (robotangle < 0) {
				robotangle += 360;
			}
			float dif = theta - robotangle; // left is negative

			if (ABS(dif)<(3)) {
				return 0;
			}

			if (dif<=-180) {
				dif += 360;
			}
			if (dif>180) {
				dif -= 360;
			}

			float speed = ((dif/180)*(TurnMax-TurnMin));

			if (speed<0) {
				speed -= TurnMin;
			}
			else if (speed>0) {
				speed += TurnMin;
			}

			return speed;

		}

		bool IsAutonomoose() {
			return IsAutonomous();
		}

		void Autonomous() { // called during autonomous
//			loaderstop.Set(.56);
//			hopperstop.Set(.5);\\

			//autoComplete == true;
			/*
			 * Maybe straight gear?
			CanMechanum(0, .5, 0, gyro.GetAngle());
			Wait(.6);
			CanMechanum(0, .3, 0, gyro.GetAngle());
			Wait(3);
			CanMechanum(.15, .2, 0, gyro.GetAngle());
			Wait(1);
			CanMechanum(-.15, .2, 0, gyro.GetAngle());
			Wait(1);
			CanMechanum(.15, .2, 0, gyro.GetAngle());
			Wait(1);
			CanMechanum(-.15, .2, 0, gyro.GetAngle());
			Wait(1);
			CanMechanum(0, .2, 0, gyro.GetAngle());
			*/

			/*
			 * straight for 5 pts
			CanMechanum(0, .5, 0, 0);
			Wait(2);
			SetAll(0);
			*/

			/*
			 * Angled gear?
			*/


//			//m_loader.Set(-.5);
//			CanMechanum(0,.5,0,gyro.GetAngle());
//			Wait(1);
//			//m_climber.Set(-1); // Never set this to a positive value
//			//m_loader.Set(0);
//			SetAll(0);
//			Wait(.7);
////			while (ABS(TurnTo(-30)) != 0 && IsAutonomous()) {
////				CanMechanum(0,0,TurnTo(-30),gyro.GetAngle());
////			}
//
//			SetAll(0);
//			Wait(.5);
//			CanMechanum(0,.3,0,0);
//			Wait(.8);
//			//m_climber.Set(0);
//
//			SetAll(0);
			/*
			 *
			 *
			*/
			tanks4nothing(-.5, -.5);
			Wait(2);
			tanks4nothing(0, 0);
			/*

			CanMechanum(0,.35,0,gyro.GetAngle());
			Wait(1.5);

			SetAll(0);
			Wait(.7);
			while ((TurnTo(140) != 0) && IsAutonomous()) {
				CanMechanum(0,0,TurnTo(140),gyro.GetAngle());
			}
			SetAll(0);
			Wait(.7);
			AutoTimer.Reset();
			AutoTimer.Start();

			while (IsAutonomoose()) {
				CAM();

				if (AutoTimer.Get() >= .5){
					hopperstop.Set(1);
					m_agitator.Set(-.7);

					AutoTimer.Stop();
				}
				else if (AutoTimer.Get() >= .4){
					hopperstop.Set(0);
				}

				CanMechanum((AdjustDistance()), 0, BoilerSpin(), 0);
				int DesiredEncoderVal = GetDesiredEncoder();//6000;
				if (visionX.size() > 0) {
					DesiredEncoderVal = GetDesiredEncoder();
				}
				SetShooter(DesiredEncoderVal);

				Wait(.005);
			}
			m_climber.Set(0);
			*/

		}

private:

	// Drive Motors
//		CanTalonSRX m_rf { 13 };
//		CanTalonSRX m_rr { 23 };
//		CanTalonSRX m_lf { 22 };
//		CanTalonSRX m_lr { 24 };

		Talon m_rf { 2 };
		Talon m_lf { 0 };
		Talon m_rr { 3 };
		Talon m_lr { 1 };
	// Accessory Motors
	CanTalonSRX m_shooter { 27 };
	CanTalonSRX m_loader { 26 };
	CanTalonSRX m_agitator { 25 };
	CanTalonSRX m_climber { 20 }; // def not 13 //NEVER SET TO A POSITIVE VALUE
	frc::Servo hopperstop { 0 };
	frc::Servo loaderstop { 1 };

	//truman is cool
	//but truman is cooler!


	// Joysticks
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	frc::Joystick opstick { 2 };

	// Timers
	frc::Timer walter; // Lets shooting motor speed up
	frc::Timer herbert; // stops agitator before shooting motor
	frc::Timer fatherTime;
	frc::Timer AutoTimer;

	// Gyro
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };

	// GRIP network table
	std::shared_ptr<NetworkTable> table { NetworkTable::GetTable("GRIP/BoilerReport1") };

	// Encoder
	Encoder *sampleEncoder = new Encoder(0,1, false, Encoder::EncodingType::k2X);
	Compressor *comp  = new Compressor(0);
	frc::DoubleSolenoid m_doubleSolenoid { 0, 1 };
	//I2C initialization
	I2C Wire { I2C::Port::kOnboard,1 };


};

START_ROBOT_CLASS(Robot)













