
#include "URGThreads.h"


class URG
{
	private:
		laser LL;
		pthread_t threads[5]; 
		struct sockaddr_in echoserver;
		unsigned int echolen;
	
	public:
		URG(const char* PORT2);		//URG U1("/dev/ttyUSB1");
		void URGStart();				//U1.URGStart();
		void URGEnd();					//U1.URGEnd();
		void URGRestart();
		void URGDisplayOn();	
		void URGDisplayOff();	
		double URGClosestLeft();
		double URGClosestRight();
		double URGStraight();												
		double getrange(int step); //double range;						//Distance: 20 - 4000mm
											//range = U1.GetRange(356);		//Step Range: 44 - 725
};																						//Angular Resalution: .36
																						//Scans from right to left



URG::URG(const char* Port2)
{
	printf("Initializing Roomba\n");
	LL.fd = -1;
	LL.Port = Port2;
	LL.end = false;
	LL.Disp = false;
	for(int i = 0; i < 683; i++) LL.data[i] = 0;
	LL.left = 0;
	LL.right = 0;
	LL.straight = 0;
		
}



void URG::URGStart()
{
	struct termios term;
	int flags;
	  
	printf("Opening connection to URG on %s\n", LL.Port);

	fflush(stdout);
	
	if((LL.fd = open(LL.Port, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
	{
		Die("Failed to Open URG\n");
	}
	if(tcflush(LL.fd, TCIFLUSH) < 0 )
	{
		Die("Failed to Flush URG\n");
	}
	if(tcgetattr(LL.fd, &term) < 0 )
	{
		Die("Failed to connect with URG\n");
	}
	
	cfmakeraw(&term);
	cfsetispeed(&term, B19200);
	cfsetospeed(&term, B19200);
	
	if(tcsetattr(LL.fd, TCSAFLUSH, &term) < 0 )
	{
		Die("Failed to Flush URG\n");
	}

   int t=0;
	int rc = pthread_create(&threads[0], NULL, URGThread, (void *) &LL );
	
	usleep(200000);
	
	return;
}			



void URG::URGEnd()
{
	LL.Disp = false;
	LL.end = true;
	sleep(2);
	close(LL.fd);
}


																	
double URG::getrange(int step)
{
	return LL.data[step];
}	

void URG::URGDisplayOn()
{
	
	LL.Disp = true;
	
	int t=1;
	int rc = pthread_create(&threads[0], NULL, URGDisplayThread, (void *) &LL );
	
	usleep(200000);
}	

void URG::URGDisplayOff()
{
	
	LL.Disp = false;
	
	usleep(200000);
}	

double URG::URGClosestLeft()
{
		return LL.left;
}
		

double URG::URGClosestRight()
{
	return LL.right;
}
		
		
double URG::URGStraight()
{
	return LL.straight;
}

void URG::URGRestart()
{
	URGEnd();
	LL.end = false;
	LL.fd = 0;
	usleep(300000);
	URGStart();	
}





									