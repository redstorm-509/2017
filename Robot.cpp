#include <WPILib.h>
#include <CanTalonSRX.h>
#include <math.h>

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
	void OperatorControl() {
		while (IsOperatorControl() && IsEnabled()) {

			comp->SetClosedLoopControl(true);

			float PovVal = guitar.GetPOV(); //trigger for strumming 180 = down 0 = up
			float magnitude = guitar.GetTwist();
			bool greenFret = guitar.GetRawButton(1);
			bool redFret = guitar.GetRawButton(2);
			bool yellowFret = guitar.GetRawButton(4);
			bool blueFret = guitar.GetRawButton(3);
			int inverse = 1;
			int solVal = 1;
			float soltime = .1;
			bool open = false;
			bool op1 = false;

			if (PovVal == 180){
				inverse = 1;
			}
			else if (PovVal == 0){
				inverse = -1;
			}
			else if (greenFret||redFret||yellowFret||blueFret){
				inverse = 1;
			}
			else {
				inverse = 0;
			}

			if (!(greenFret||redFret||yellowFret||blueFret)){
				//drive with stum
				m_rf.Set(inverse*magnitude);
				m_rr.Set(inverse*magnitude);
				m_lf.Set(-inverse*magnitude);
				m_lr.Set(-inverse*magnitude);
			}
			else if (greenFret){
				m_rf.Set(-inverse*magnitude/2);
				m_rr.Set(-inverse*magnitude/2);
				m_lf.Set(-inverse*magnitude/2);
				m_lr.Set(-inverse*magnitude/2);

			}
			else if (redFret){
				m_rf.Set(inverse*magnitude/2);
				m_rr.Set(inverse*magnitude/2);
				m_lf.Set(inverse*magnitude/2);
				m_lr.Set(inverse*magnitude/2);

			}
			else if (blueFret){
				m_rf.Set(-inverse*magnitude/2);
				m_rr.Set(inverse*magnitude/2);
				m_lf.Set(-inverse*magnitude/2);
				m_lr.Set(inverse*magnitude/2);

			}
			else if (yellowFret){
				m_rf.Set(inverse*magnitude/2);
				m_rr.Set(-inverse*magnitude/2);
				m_lf.Set(inverse*magnitude/2);
				m_lr.Set(-inverse*magnitude/2);

			}
			if (guitar.GetRawButton(5)){
				if (op1 == false){
					op1 = true;
				}
			}
			if (op1 && !open) {
				sol.Set(solVal);
				SolTimer.Start();
				SolButtCool.Start();
				open = true;
			}
			if (SolTimer.Get() > soltime){
				sol.Set(0);
				SolTimer.Stop();
				SolTimer.Reset();
				open = false;
			}
			if (SolButtCool.Get() > (soltime + 1)){
				op1 = false;
				SolButtCool.Stop();
				SolButtCool.Reset();
			}
			frc::Wait(kUpdatePeriod);  // Wait 5ms for the next update.
		}
	}

private:
	// Initialize Joystick on port 0
	frc::Joystick guitar { 3 };
	frc::Joystick opstick { 2 };


	/* The motor to control with the Joystick. This uses a Talon speed
	 * controller; use the Victor or Jaguar classes for other speed controllers.
	 *
	 * Initialize the Talon on channel 0
	 */
	Talon m_rf { 2 };
	Talon m_lf { 0 };
	Talon m_rr { 3 };
	Talon m_lr { 1 };
	Talon sol { 4 };

	Compressor *comp  = new Compressor(0);

	frc::Timer SolTimer;
	frc::Timer SolButtCool;
	// Update every 0.005 seconds/5 milliseconds.
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)
