/*
 * Read through the comments at the beginning of each section for notes on what the
 * section of code does, and how to use it. Within each section will be comments on how
 * the individual lines of code actually work. I'll try to not be too sarcastic and not
 * swear too much in my comments :)
 */

//========================== BEGIN INCLUDES =========================//

/*
 * These are the header files that are required for the code in this file to run.
 * Not all of these files will always be used, you only need to include files that
 * are required for the drive system you want.
 *
 * WPILib.h - This is just a header file that includes a bunch of useful stuff for you.
 * If you open up the header file you can see all the handy stuff included
 *
 * CanTalonSRX.h - This header file is required to interact with the CAN Talon SRX
 * motor controller subsystem. If you want to use a different motor controller, include
 * the corresponding header file
 *
 * math.h - This is used to perform mathematical operations such as sin() cos() sqrt() etc
 */

#include <WPILib.h>
#include <CanTalonSRX.h>
#include <math.h>

//=========================== END INCLUDES ====================================//

class Robot: public frc::SampleRobot {
public:

	//======================== BEGIN ABS =====================//

	/*
	 * This is an absolute value function. The absolute value function built into math.h
	 * is TERRIBLE, because it will only return an integer value. Use this to return a
	 * float value, which is what we use most. This function is used in some drive
	 * systems in this file.
	 */

	float ABS(float input) {
		if (input<0) {
			input = input * -1;
		}
		return input;
	}
	//======================== END ABS =======================//

	//================== BEGIN MECANUM_CARTESIAN =================//

	/*
	 * This function takes joystick inputs, as well as a gyro angle input, and
	 * sets the motors. The code is heavily commented for your convenience,
	 * feel free to remove the comments once you understand the code. It assumes
	 * the four motors are named:
	 *
	 * rfm - Front right motor
	 * rrm - Rear right motor
	 * lfm - Front left motor
	 * lrm - Rear left motor
	 *
	 * If you want to run the mecanum drive without the gyro, just set the parameter to 0
	 *
	 * EX. with gyro
	 * 		Drive_Mecanum_Cartesian(rstick.GetX(),rstick.GetY(),lstick.GetX(),gyro.GetAngle())
	 *
	 * EX. without gyro
	 * 		Drive_Mecanum_Cartesian(rstick.GetX(),rstick.GetY(),lstick.GetX(),0.0)
	 */

	void Drive_Mecanum_Cartesian(float RX, float RY, float LX, float robotangle){
		float PI = 3.141592;
		RX = -RX; // Either RY or RX needed to be inverted, I chose RX
		float mag = sqrt((RY*RY)+(RX*RX)); // This is the hypotenuse of the triangle made by RX and RY
		robotangle = (((PI)/180)*robotangle); // All math functions are in radians, so I converted robotangle to radians
		// the below while loop standardizes the robot angle to be between 0 and (2*PI), equal to 0 to 360 degrees
		while ((robotangle >= (2*PI)) or (robotangle < 0)){
			if (robotangle >= (2*PI)){
				robotangle = robotangle - (2*PI);
			}

			else if (robotangle < 0){
				robotangle = robotangle + (2*PI);
			}
		}
		float joyangle = 0; // defining the variable, 0 means nothing
		float dif = 0; // defining the variable, 0 means nothing
		/*
		 * The following if statements alter the RY and RX values in accordance to the
		 * gyro rotation. For example, if the robot is pointing 90 degrees to the right,
		 * pushing directly up on the joystick would be changed to RY = 0 and RX = -1,
		 * as if the joystick were being pushed 90 degrees to the left.
		 *
		 * Yah atan() is pretty cool.
		 */
		if ((RX>=0) && (RY>0)) { // Top Right Quadrant
			joyangle = atan(RX/RY);
			dif = joyangle - robotangle;
			RY = mag*cos(dif);
			RX = mag*sin(dif);
		}
		else if ((RX>0) && (RY<=0)) { // Bottom Right Quadrant
			joyangle = (-atan(RY/RX)) + ((PI)/2);
			dif = joyangle - robotangle;
			RX = mag*sin(dif);
			RY = mag*cos(dif);
		}
		else if ((RX<=0) && (RY<0)) { // Bottom Left Quadrant
			joyangle = atan(RX/RY) + (PI);
			dif = joyangle - robotangle;
			RY = mag*cos(dif);
			RX = mag*sin(dif);
		}
		else if ((RX<0) && (RY>=0)) { // Top Left Quadrant
			joyangle = atan(RY/RX) + ((3*PI)/2);
			dif = joyangle - robotangle;
			RX = mag*sin(dif);
			RY = -mag*cos(dif);
		}

		mag = sqrt((RY*RY)+(RX*RX)); // get a new magnitude value for the new RX and RY

		/*
		 * The following equations assign 4 variables, representing 4 motors, values.
		 * These values will at points exceed 1, so they cannot be set directly to the motors
		 */
		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX - RX;
		float rrVal = -RY + LX + RX;

		/*
		 * Since the values above range from -1 to +2, we parse through each variable
		 * and if the highest is greater than 1, then set it to 1 and scale each other
		 * variable down proportionally. (The end range will be proportionally the same
		 * but in a range from -1 to 1). The values are also multiplied by the
		 * magnitude/sqrt(2), which gives better scaling of the robot speed. The reason
		 * it's divided by sqrt(2) is because the maximum value for the magnitude is
		 * sqrt(2), giving the magnitude a range from 0 to 1.
		 */
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

		// And finally set the motors to their respective variables :)
		lfm.Set(lfVal);
		lrm.Set(lrVal);
		rfm.Set(rfVal);
		rrm.Set(rrVal);
	}
	//======================== END MECANUM_CARTESIAN ========================//

	void OperatorControl() {
		while (IsOperatorControl() && IsEnabled()) {

			/*
			 * Following are examples on how to use each drive system. I would not advise
			 * running all drive systems at the same time. That would be bad.
			 */
			//Drive_Mecanum_Cartesian(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), gyro.GetAngle());

			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.
		}
	}

private:
	/*
	 * These are the CAN IDs which will be used. They are set to static variables in order
	 * to have lengthy names that are easier to read, when changing the CAN IDs
	 */
	static constexpr int kFrontLeftChannel = 20;
	static constexpr int kRearLeftChannel = 21;
	static constexpr int kFrontRightChannel = 22;
	static constexpr int kRearRightChannel = 23;

	// "CanTalonSRX" determines the type of speed controller that is used
	CanTalonSRX rfm { kFrontRightChannel };
	CanTalonSRX rrm { kRearRightChannel };
	CanTalonSRX lfm { kFrontLeftChannel };
	CanTalonSRX lrm { kRearLeftChannel };

	/*
	 * Sets up 2 joysticks named lstick and rstick in IDs 0 and 1. You should change which
	 * joystick is in which port from the drive station, not the robot code.
	 */
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };

	// This initialized an ADXRS450 gyro in the built in SPI port on the roborio
	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };

	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
