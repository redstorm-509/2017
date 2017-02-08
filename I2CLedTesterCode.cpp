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
				ChangeColor(2); //red team
			}
			else if(opstick.GetRawButton(2)){
				ChangeColor(3); //blue team
			}
			else if (opstick.GetRawButton(3)){
				ChangeColor(7); //has target
			}
			else if (opstick.GetRawButton(4)){
				ChangeColor(8); //no target
			}
			else if(opstick.GetRawButton(6)){
				ChangeColor(4); //loading
			}
			else if(opstick.GetRawButton(7)){
				ChangeColor(5); //shooting
			}
			else {
				ChangeColor(6); //end action
			}

		}
		ChangeColor(1); //rainbow
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
