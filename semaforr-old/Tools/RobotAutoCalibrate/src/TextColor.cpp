#include "TextColor.h"
	////////////////////////////////////////////////////////////////////////
	//////////////////////// Colored Console Text //////////////////////////
	void textcolor(int attr, int fg, int bg)
	{	char command[13];
	
		sprintf(command, "%c[%d;%d;%dm", 0x1B, attr, fg + 30, bg + 40);
		printf("%s", command);
	}
	void RED()
	{	
		textcolor(1, 1, 8);
	}
	void WHITE()
	{	
		textcolor(0, 7, 8);
	}
	void GREEN()
	{	
		textcolor(1, 2, 8);	
	}
	void BLUE()
	{	
		textcolor(1, 4, 8);	
	}
	void TBLUE(string message)
	{	
		textcolor(1, 4, 8);
		//cout << message;
		printf("%s",message.c_str());
		//puts(message.c_str());
		//puts(message.c_str());
		textcolor(0, 7, 8);
	}
	void TRED(string message)
	{	
		textcolor(1, 1, 8);
		//cout << message;
		//printf("%s",message);
		//puts(message.c_str());
		printf("%s",message.c_str());
		textcolor(0, 7, 8);
	}
	void TGREEN(string message)
	{	
		textcolor(1, 2, 8);
		//cout << message;
		//printf("%s",message);
		//puts(message.c_str());
		printf("%s",message.c_str());
		textcolor(0, 7, 8);
	}
	//////////////////////// Colored Console Text //////////////////////////
	////////////////////////////////////////////////////////////////////////

	
	
	string itoa(int n)
	{
		string str;
		char cstr[10];
		//cout << "\ntest2\n";
		sprintf(cstr, "%d", n);
		//cout << "\ntest3\n";
		str = cstr;
		//cout << "\ntest4\n";
		return str;
	}
	void emptyfunc()
	{
		 TGREEN("");
		TBLUE("");
		TRED("");
		RED();
		WHITE();
		GREEN();
		BLUE();
		itoa(0);
		emptyfunc();
		textcolor(0,0,0);		
	}

