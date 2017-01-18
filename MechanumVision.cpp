//mechanum vision
#include "WPILib.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

    // Channels for the wheels
    const static int frontLeftChannel	= 2;
    const static int rearLeftChannel	= 3;
    const static int frontRightChannel	= 0;
    const static int rearRightChannel	= 1;

    const static int joystickChannel	= 0;

	RobotDrive robotDrive;	// robot drive system
	Joystick rstick;			// only joystick
	Joystick lstick;
	ADXRS450_Gyro gyro; // A gyro. Duh.

	std::shared_ptr<NetworkTable> table;

		IMAQdxSession session; //session
		Image *frame; //camera frame
		IMAQdxError imaqError; //imaq
		std::unique_ptr<AxisCamera> camera; //camera

	// update every 0.005 seconds/5 milliseconds.
	float camcenterx = 153;
	std::vector<double> visionX;
	std::vector<double> visionY;

	float Snotarget = .2;
	float Sminimum = .05;
	float desireddistance = 80;
	float distmaxspeed = .3;
	float distminspeed = .02;


public:
	Robot() :
			robotDrive(frontLeftChannel, rearLeftChannel,
					   frontRightChannel, rearRightChannel),	// these must be initialized in the same order
			rstick(joystickChannel),								// as they are declared above.
			lstick(1),
			gyro(SPI::Port::kOnboardCS0),
			table(NetworkTable::GetTable("GRIP/BoilerReport1"))
			{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
	}

	void RobotInit() override {
		{	CameraServer::GetInstance()->SetQuality(50);
			CameraServer::GetInstance()->StartAutomaticCapture("cam0");
			CameraServer::GetInstance()->StartAutomaticCapture("cam1");
			CameraServer::GetInstance()->StartAutomaticCapture("cam2");}
			// create an image
			frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
			// open the camera at the IP address assigned. This is the IP address that the camera
			// can be accessed through the web interface.
			camera.reset(new AxisCamera("axis-camera.local"));
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

	void TargetBoiler() {
			bool centered = false;
			while ((not(centered)) && (IsOperatorControl() && IsEnabled())) {
				if (lstick.GetRawButton(1)) {
					charge();
				}
				float RM = 0;
				float LM = 0;
				if ((visionX.size()) < 1) {
					RM = -.2;
					LM = .2;
				}
				else {
					if (ABS(camcenterx - visionX[0])>10) {
						float speed = (((camcenterx - visionX[0])/camcenterx)/5);
						if (speed<0) {
							speed = -.15;
						}
						else {
							speed = .15;
						}
						RM = -(speed);
						LM = speed;
					}
					else {
						centered = true;
					}
				}
				robotDrive.SetLeftRightMotorOutputs(-LM,RM);
				CAM();
				Wait(0.005);
			}
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

	void charge() {
			robotDrive.SetLeftRightMotorOutputs(-.8,.8);
			Wait(lstick.GetZ()+.5);
		}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()
	{
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
			camera->GetImage(frame);
						imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
						CameraServer::GetInstance()->SetImage(frame);

			if (rstick.GetRawButton(3)) {
				gyro.Reset();
			}
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.


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

			SmartDashboard::PutNumber("TargetHeight",GetDistance());

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

};

START_ROBOT_CLASS(Robot)
