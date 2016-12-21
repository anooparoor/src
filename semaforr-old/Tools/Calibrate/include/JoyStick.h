
#include "JoyStickThreads.h"


class JoyStick
{
	private:
		Joy SS;
		pthread_t threads[1]; 
	
	public:
		JoyStick(const char* DEV);	//JoyStick JS("/dev/input/js0");
		void JoyStart();				//JS.JoyStart();
		void JoyEnd();					//JS.JoyEnd();
		void JoyRestart();
		double GetX();					//JS.GetX();
		double GetY();					//JS.GetY();
		double GetButtons();			//JS.GetButtons();
};

//////////////////////////////////////// JoyStick() ///////////////////////////////

JoyStick::JoyStick(const char* DEV)
{
	printf("Initializing JoyStick\n");
	SS.JOY_DEV = DEV;
  	SS.buttonsOn = false;
  	SS.joyX = 0;
  	SS.joyY = 0;
  	SS.buttons = 0;
	SS.end = false;
}

//////////////////////////////////////// JoyStart() /////////////////////////////

void JoyStick::JoyStart()
{
	printf("Connecting to JoyStick on %s\n", SS.JOY_DEV);
	int t=0;
	int rc = pthread_create(&threads[0], NULL, JoyThread, (void *) &SS);
	usleep(200000);
	return;
}

//////////////////////////////////////// JoyEnd() ///////////////////////////////

void JoyStick::JoyEnd()
{
	SS.end = true;
}

//////////////////////////////////////// GetX() /////////////////////////////////

double JoyStick::GetX()
{
	return SS.joyX;
}

//////////////////////////////////////// GetY() /////////////////////////////////

double JoyStick::GetY()
{
	return SS.joyY;
}

//////////////////////////////////////// GetButtons() ///////////////////////////

double JoyStick::GetButtons()
{
	return SS.buttons;
}

void JoyStick::JoyRestart()
{
	JoyEnd();
	SS.end = false;
	usleep(300000);
	JoyStart();	
}
