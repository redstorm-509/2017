#include <SampleRobot.h>
#include <I2C.h>
#include <Timer.h>
#include <Talon.h>
#include <WPILib.h>

class Robot: public frc::SampleRobot {
public:

	void ChangeColor(uint8_t val) {
		Wire.Transaction(&val,1,NULL,0);
	}

	void OperatorControl() {
		while (IsOperatorControl() && IsEnabled()) {

			if (opstick.GetRawButton(1)){
				ChangeColor(2);
			}
			else if(opstick.GetRawButton(2)){
				ChangeColor(3);
			}
			else if (opstick.GetRawButton(3)){
				ChangeColor(7);
			}
			else if (opstick.GetRawButton(4)){
				ChangeColor(8);
			}
			else if(opstick.GetRawButton(6)){
				ChangeColor(4);
			}
			else if(opstick.GetRawButton(7)){
				ChangeColor(5);
			}
			else {
				ChangeColor(6);
			}

		}
		ChangeColor(1);
	}
private:

	I2C Wire { I2C::Port::kOnboard,1 }; //Initialize the I2C wire communication on the roborio built in I2C port and gives it an address

	// Initialize Joystick on port 0
	frc::Joystick opstick { 2 };

	frc::Talon rfm { 1 };


	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
