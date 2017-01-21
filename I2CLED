#include <SampleRobot.h>
#include <I2C.h>
#include <Timer.h>
#include <Talon.h>

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
	/**
	 * Runs the motor from the output of a Joystick.
	 */
	private:
	// Initialize Joystick on port 0
	//frc::Joystick m_stick { 0 };

	void ChangeColor(uint8_t val) {
		Wire.Transaction(&val,1,NULL,0);
	}

	void OperatorControl() {
	//	while (IsOperatorControl() && IsEnabled()) {
			/* Set the motor controller's output. This takes a number from -1
			 * (100% speed in reverse) to +1 (100% speed forwards).
			 */
			//int whatever = NULL;
			//uint8_t value;

			ChangeColor(1);

			Wait(15);
			ChangeColor(3);
		/*
			uint8_t value = 1; // blue
			//uint8_t* value = &num;
			//rfm.Set(.5);
			Wire.Transaction(&value,1,NULL,0);

			//rfm.Set(.9);
			Wait(15);

			value = 2; // red
			//rfm.Set(0);
			//Wire.Write(value,value);
			Wire.Transaction(&value,1,NULL,0);
			//rfm.Set(-.9);
			/*Wait(.5);

			value = 3; // unknown
			//Wire.Write(value,value);
			Wire.Transaction(&value,1,NULL,0);
			Wait(.5);

			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.*/
	//	}
	}


I2C Wire { I2C::Port::kOnboard,1 };

	/* The motor to control with the Joystick. This uses a Talon speed
	 * controller; use the Victor or Jaguar classes for other speed controllers.
	 *
	 * Initialize the Talon on channel 0
	 */

	frc::Talon rfm { 1 };


	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
