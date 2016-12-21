#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <sys/time.h>
#include <stdio.h>
#include <time.h>

#include "include/Robots.h"

using namespace std;
using namespace PlayerCc;

PlayerClient    robot;
Position2dProxy pp(&robot, 0);

bool inMotion()
{
  return pp.GetXSpeed() || pp.GetYSpeed() || pp.GetYawSpeed();
}


double X = 0;
double Y = 0;
double Buttons = 0;

float move = 0;

int main(int argc, char **argv)
{
 
 JoyStick JJ("/dev/input/js0");
	JJ.JoyStart();
	
	double turn = 0;
	double Trate = 0;
	double Dmove = 0;

 	while(Buttons != 240)
	{
  		Buttons = JJ.GetButtons();
  		
  		if(Buttons == 1)
  		{
  			cout << "\n22.5%\n";
  			turn = M_PI_2 / 4;
  			cout << "\n5cm\n";
  			move = 5;
  		}
  		if(Buttons == 2)
  		{
  			cout << "\n45%\n";
  			turn = M_PI_2 / 2;
  			cout << "\n10cm\n";
  			move = 10;
  		}
  		if(Buttons == 4)
  		{
  			cout << "\n90%\n";
  			turn = M_PI_2;
  			cout << "\n15cm\n";
  			move = 15;
  		}
  		if(Buttons == 8)
  		{
  			cout << "\n135%\n";
  			turn = M_PI_2 * 1.5;
  			cout << "\n30cm\n";
  			move = 30;
  		}
		if(Buttons == 16)
  		{
  			cout << "\n180%\n";
  			turn = M_PI_2 * 2;
  			cout << "\n45cm\n";
  			move = 45;
  		}
		if(Buttons == 32)
  		{
  			cout << "\n360%\n";
  			turn = M_PI_2 * 4;
  			cout << "\n60cm\n";
  			move = 60;
  		}
  		if(Buttons == 128)
  		{
  			cout << "\nEnter X: ";
  			cin >> X;
  			cout << "\n";
  			
  			cout << "\nEnter Y: ";
  			cin >> Y;
  			cout << "\n";
  			
  			cout << "\nEnter Final Theta in Degrees: ";
  			cin >> Trate;
  			cout << "\n";
  			
  			Trate = Trate / 90;
  			turn = M_PI_2 * Trate;
  			move = 0;
  			pp.ResetOdometry();
	  		pp.GoTo(X, Y, turn);
  		}
  		if(Buttons == 256)
  		{
  			cout << "\nEnter Turn Rate in Degrees: ";
  			cin >> Trate;
  			cout << "\n";
  			Trate = Trate / 90;
  			turn = M_PI_2 * Trate;
  			move = 0;
  		}
		if(Buttons == 512)
  		{
  			cout << "\nEnter Distanec in cm: ";
  			cin >> move;
  			cout << "\n";
  		}

  		
  		
  		
  		
  		
  		

  		
  		if(JJ.GetY() == 32767)	
		{
			cout << "\nforward\n";
	  		pp.ResetOdometry();
	  		pp.GoTo(move / 100, 0, 0);
  		}
  		
  		if(JJ.GetY() == -32767)	
		{
			cout << "\n50/50\n";
	  		pp.ResetOdometry();
	  		pp.GoTo(.50, -.50, 0);
  		}
  		
  		if(JJ.GetX() == 32767)	
		{
			cout << "\nturn left\n";
	  		pp.ResetOdometry();
	  		pp.GoTo(0, 0, turn);
  		}
  		if(JJ.GetX() == -32767)	
		{
			cout << "\nturn right\n";
	  		pp.ResetOdometry();
	  		pp.GoTo(0, 0, turn * -1);
  		}
  		
  		usleep(250000);
 	}
  return 0;
}
