#ifndef Text_Color_H_
#define Text_Color_H_


#include <string>
//#include <string.h>
#include <stdio.h>
//#include <termios.h>
//#include <math.h>
//#include <iostream>

using namespace std;


//#include "TextColor.h"
//example: TBLUE(M + "\n\n" + " " + " This is a test " + itoa(525) + "OK\n");




	
	//string M = "";

	////////////////////////////////////////////////////////////////////////
	//////////////////////// Colored Console Text //////////////////////////
	void textcolor(int attr, int fg, int bg);
	void RED();
	void WHITE();
	void GREEN();
	void BLUE();
	void TBLUE(string message);
	void TRED(string message);
	void TGREEN(string message);
	//////////////////////// Colored Console Text //////////////////////////
	////////////////////////////////////////////////////////////////////////

	
	
	string itoa(int n);
	void emptyfunc();




#endif

