#include <SampleRobot.h>
#include <I2C.h>
#include <Timer.h>
#include <Talon.h>

class Robot: public frc::SampleRobot {
public:

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


	I2C Wire { I2C::Port::kOnboard,1 }; //Initialize the I2C wire communication on the roborio built in I2C port and gives it an address 

	
	frc::Talon rfm { 1 };


	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
