
#include "URG_UG01Threads.h"

class URG_UG01
{
	private:
		UG01laser LL;
		pthread_t threads[5]; 
		struct sockaddr_in echoserver;
		unsigned int echolen;
	
	public:
		URG_UG01(const char* PORT2);		//URG U1("/dev/ttyUSB1");
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
																						//Angular Resalution: .36
	
	
	};
	
	
			
URG_UG01::URG_UG01(const char* Port2)
{
	printf("Initializing URG Laser\n");
	LL.fd = -1;
	LL.Port = Port2;
	LL.end = false;
	LL.Disp = false;
	for(int i = 0; i < 683; i++) LL.data[i] = 0;
	LL.left = 0;
	LL.right = 0;
	LL.straight = 0;		
}


																			//Scans from right to left

void URG_UG01::URGStart()
{
	struct termios term;
	int flags;
	int numread = 0;
	  
	printf("Opening connection to URG on %s\n", LL.Port);

	fflush(stdout);
	
	if((LL.fd = open(LL.Port, O_RDWR, S_IRUSR | S_IWUSR )) < 0 )
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
	
	
	//////////////////// Starts URG Laser //////////////////
	char startURG[4];
	sprintf(startURG, "BM\n\0");
	char reply[24];
	sprintf(reply, "                       \0");	
	cout << "Starting URG Laser\n\n";
	sprintf(reply, "                       \0");
	while(reply[0] != 'B')
	{
		usleep(100000);
		if(write(LL.fd, startURG, 3) < 0)
		{
			perror("create_get_sensors():write():");
			return;
		}
	
		numread = read(LL.fd,reply,23);
	}
	cout << "\nURG Laser Started\n\n";
	//////////////////// Starts URG Laser //////////////////
	
	
	usleep(200000);

   int t=0;
	int rc = pthread_create(&threads[0], NULL, UG01Thread, (void *) &LL );
	
	usleep(200000);
	
	return;
}			




void URG_UG01::URGEnd()
{
	LL.Disp = false;
	LL.end = true;
	sleep(2);
	close(LL.fd);
}

																	
double URG_UG01::getrange(int step)
{
	return LL.data[step];
}	

void URG_UG01::URGDisplayOn()
{
	
	LL.Disp = true;
	
	int t=1;
	int rc = pthread_create(&threads[0], NULL, URGDisplayThread, (void *) &LL );
	
	usleep(200000);
}	

void URG_UG01::URGDisplayOff()
{
	
	LL.Disp = false;
	
	usleep(200000);
}	

double URG_UG01::URGClosestLeft()
{
		return LL.left;
}
		

double URG_UG01::URGClosestRight()
{
	return LL.right;
}
		
		
double URG_UG01::URGStraight()
{
	return LL.straight;
}

void URG_UG01::URGRestart()
{
	URGEnd();
	LL.end = false;
	LL.fd = 0;
	usleep(300000);
	URGStart();	
}





									