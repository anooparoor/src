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

 	while(Buttons != 240)
	{
  		Buttons = JJ.GetButtons();
  		
  		if(Buttons == 1)
  		{
  			cout << "\n180%\n";
  			turn = M_PI_2 * 2;
  			cout << "\n5cm\n";
  			move = 5;
  			
  		}
  		if(Buttons == 2)
  		{
  			cout << "\n90%\n";
  			turn = M_PI_2;
  			cout << "\n10cm\n";
  			move = 10;
  		}
  		if(Buttons == 4)
  		{
  			cout << "\n45%\n";
  			turn = M_PI_2 / 2;
  			cout << "\n10cm\n";
  			move = 10;
  		}
  		if(Buttons == 8)
  		{
  			cout << "\n22.5%\n";
  			turn = M_PI_2 / 4;
  			cout << "\n15cm\n";
  			move = 15;
  		}
  		if(Buttons == 16)
  		{
  			cout << "\n135%\n";
  			turn = M_PI_2 * 1.5;
  			cout << "\n30cm\n";
  			move = 30;
  		}
  		if(Buttons == 32)
  		{
  			cout << "\n135%\n";
  			turn = M_PI_2 * 1.5;
  			cout << "\n60cm\n";
  			move = 60;
  		}
  		if(Buttons == 64)
  		{
  			cout << "\n135%\n";
  			turn = M_PI_2 * 1.5;
  			cout << "\n45cm\n";
  			move = 45;
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
  		
  		usleep(150000);
 	}
  return 0;
}
