#include <Joystick.h>
#include <SampleRobot.h>
#include <Talon.h>
#include <Timer.h>
#include <WPILib.h>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together. The program also delays a
 * short time in the loop to allow other threads to run. This is generally a
 * good idea, especially since the joystick values are only transmitted from the
 * Driver Station once every 20ms.
 */
class Robot: public frc::SampleRobot {
public:


	float PI = 3.14159;

	/**
	 * Runs the motor from the output of a Joystick.
	 */

	float ABS(float input) {
		if (input<0) {
			input = input * -1;
		}
		return input;
	}

	void CanMechanum(float RX, float RY, float LX, float robotangle){
		float mag = sqrt((RY*RY)+(RX*RX));
		float joyangle = 0;
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

	void OperatorControl() {
		while (IsOperatorControl() && IsEnabled()) {
			/* Set the motor controller's output. This takes a number from -1
			 * (100% speed in reverse) to +1 (100% speed forwards).
			 */

			if (opstick.GetRawButton(4)) {
				gyro.Reset();
			}

			if (opstick.GetThrottle()>.2) {
				CanMechanum(opstick.GetX(), -opstick.GetY(), (opstick.GetThrottle()/2), gyro.GetAngle());
			}
			else {
				CanMechanum(opstick.GetX(), -opstick.GetY(), -(opstick.GetTwist()/2), gyro.GetAngle());
			}


			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.
		}
	}

private:
	Talon m_rf { 2 };
	Talon m_lf { 0 };
	Talon m_rr { 3 };
	Talon m_lr { 1 };
	// Accessory Motors

	// Joysticks
	frc::Joystick opstick { 2 };

	ADXRS450_Gyro gyro { SPI::Port::kOnboardCS0 };

	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
