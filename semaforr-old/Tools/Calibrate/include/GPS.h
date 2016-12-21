
#include "GPSThreads.h"


class GPS
{
	private:
		sGPS GG;
		pthread_t threads[5]; 
		struct sockaddr_in echoserver;
		unsigned int echolen;
	
	public:
		GPS(const char* PORT2);	
		void GPSStart();	
		void GPSEnd();		
		void GPSRestart();
		sGPS GetGPS();	
		int GetAngle(sGPS dest);
		int GetDistance(sGPS dest);	
		void Print();								
};																				
																					
GPS::GPS(const char* Port2)
{
	printf("Initializing Roomba\n");
	GG.fd = -1;
	GG.Port = Port2;
	GG.end = false;
	GG.Inorthmin = 0;	//North minutes interger
	GG.Inorthsec = 0;	//North seconds interger
	GG.Inorthdeg= 0;	//North degrees interger
	GG.Iwestmin = 0;	//West minutes interger
	GG.Iwestsec = 0;	//West seconds interger
	GG.Iwestdeg = 0;	//West degrees interger		
	GG.northtotal = 0;
	GG.westtotal = 0;	
}


void GPS::GPSStart()
{
	struct termios term;
	int flags;
	  
	printf("Opening connection to GPS on %s...", GG.Port);
	fflush(stdout);

	
	if((GG.fd = open(GG.Port, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
	{
		Die("Failed to Open URG\n");
	}
	if(tcflush(GG.fd, TCIFLUSH) < 0 )
	{
		Die("Failed to Flush URG\n");
	}
	if(tcgetattr(GG.fd, &term) < 0 )
	{
		Die("Failed to connect with URG\n");
	}
	
	cfmakeraw(&term);
	cfsetispeed(&term, B4800);//
	cfsetospeed(&term, B4800);//
	
	if(tcsetattr(GG.fd, TCSAFLUSH, &term) < 0 )
	{
		Die("Failed to Flush URG\n");
	}

   int t=0;
	int rc = pthread_create(&threads[0], NULL, GPSThread, (void *) &GG );
	
	usleep(200000);
	
	cout << "\nWaiting For GPS Location.....\n";
	
	while(GG.northtotal == 0){}
	
	cout << "Location Found at:\n";
	Print();
	
	usleep(200000);
		
	return;
}			

void GPS::GPSEnd()
{
	GG.end = true;
	sleep(2);
	close(GG.fd);
}

void GPS::GPSRestart()
{
	
}

sGPS GPS::GetGPS()
{
	return GG;
}

void GPS::Print()
{        		
	cout << "North: " << GG.Inorthdeg << "d " << GG.Inorthmin << "." << GG.Inorthsec << "'  ";
   cout << "West: " << GG.Iwestdeg << "d " << GG.Iwestmin << "." << GG.Iwestsec << "'" << endl;
   cout << fixed << GG.northtotal << "\t\t" << GG.westtotal << endl;
}

int GPS::GetAngle(sGPS dest)
{
	double Dangle;
	int Iangle;
	double deltaN = GG.northtotal - dest.northtotal;
	double deltaW = GG.westtotal - dest.westtotal;
	if (deltaN == 0)
	{
		if (deltaW < 0) return 90;
		else return 270;
	}
	if (deltaW == 0)
	{
		if (deltaN > 0) return 0;
		else return 180;
	}
	if (deltaN < 0 && deltaW > 0)
	{
		Dangle = (atan(-1.0*deltaW/deltaN) * 180 / PI);
		Iangle = static_cast < int >(Dangle);
		//cout << endl << Dangle << endl;
		return Iangle;
	}
	if (deltaN > 0 && deltaW > 0)
	{
		Dangle = (90.00 + (atan(deltaN/deltaW) * 180 / PI));
		Iangle = static_cast < int >(Dangle);
		//cout << endl << Dangle << endl;
		return Iangle;
	}
	if (deltaN > 0 && deltaW < 0)
	{
		Dangle = (180.00 + (atan(deltaW / deltaN * -1.00) * 180 / PI) );
		Iangle = static_cast < int >(Dangle);
		//cout << endl << Dangle << endl;
		return Iangle;
	}
	if (deltaN < 0 && deltaW < 0)
	{
		Dangle = (270.00 + (atan(deltaN/deltaW)* 180 / PI));
		Iangle = static_cast < int >(Dangle);
		//cout << endl << Dangle << endl;
		return Iangle;
	}		
	return -1;
}


int GPS::GetDistance(sGPS dest)
{
	double distance, northDiff, westDiff;
	westDiff = abs(GG.westtotal - dest.westtotal);
	northDiff= abs(GG.northtotal - dest.northtotal);
	distance = sqrt(northDiff * northDiff + westDiff * westDiff);	
	return distance;
}	






