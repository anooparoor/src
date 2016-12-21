
#include "RoombaThreads.h"


class Roomba
{
	private:
		Room RR;
		pthread_t threads[5]; 
	
	public:
		Roomba(const char* Port2);	//Roomba R1("/dev/ttyUSB0");
		void RoombaStart();			//R1.RoombaStart();			
		void RoombaEnd();				//R1.RoombaEnd();		
		void RoombaRestart();					
		int GetLeftBumper();
		int GetRightBumper();
		int GetDistance();
		double GetAngle();
		int GetBattery();
		int GetRightWheel();
		int GetLeftWheel();	
		int GetCasterWheel();		
		void VacuumOn();
		void VacuumOff();							
		void SetMotors(double speed, double turnrate);//double speed = 50;			speed: (-100 - 100)
																	 //double turnrate = 1.5;		turnrate: (-3.3 - 3.3)
};																	 //R1.SetMotors(speed, turnrate);



Roomba::Roomba(const char* Port2)
{
	printf("Initializing Roomba\n");
	RR.fd = -1;
	RR.Port = Port2;
	RR.Speed = 0;
	RR.TurnRate = 0;
	RR.distance = 0;
	RR.angle = 0;
	RR.end = false;
	RR.vacuumOn = false;
	RR.vacuumOff = false;
	RR.vacuumRun = false;
	RR.leftbump = 0;
	RR.rightbump = 0;
	RR.leftwheel = 0;
	RR.rightwheel = 0;
	RR.casterwheel = 0;
	RR.battery = 0;
}



void Roomba::RoombaStart()
{
	struct termios term;
  	int flags;
  
	unsigned char cmdbuf[1];	
	
  	printf("Opening connection to Roomba on %s\n", RR.Port);
 
	fflush(stdout); 
 
	if((RR.fd = open(RR.Port, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
  	{
		Die("Failed to Open Roomba\n");
  	}
  	
  	if(tcflush(RR.fd, TCIFLUSH) < 0 )
  	{
		Die("Failed to Flush Roomba\n");
  	}
  	if(tcgetattr(RR.fd, &term) < 0 )
  	{
    	Die("Failed to connect with Roomba\n");
  	}

  	cfmakeraw(&term);
  	//cfsetispeed(&RR.term, B19200);
	//cfsetospeed(&RR.term, B19200);
	cfsetispeed(&term, B57600);
	cfsetospeed(&term, B57600);
	//cfsetispeed(&RR.term, B115200);
   //cfsetospeed(&RR.term, B115200);

	cmdbuf[0] = 128;
	
	if(write(RR.fd, cmdbuf, 1) < 0)
	{
		Die("Could Not write to Roomba\n");
 	}	
	
	usleep(200000);
	
	cmdbuf[0] = 130;
	
	if(write(RR.fd, cmdbuf, 1) < 0)
	{
		Die("Could Not write to Roomba\n");
 	}		
 	
 	usleep(200000);
	
	cmdbuf[0] = 132;
	
	write(RR.fd, cmdbuf, 1);

	usleep(200000);

   int t=0;
	int rc = pthread_create(&threads[0], NULL, RoombaThread, (void *) &RR );
	
	usleep(200000);
	
	return;
}					
		
		

void Roomba::RoombaEnd()
{
	RR.end = true;
	sleep(2);
	close(RR.fd);
}					
								
															
void Roomba::SetMotors(double speed, double turnrate)
{
	RR.Speed = speed;
	RR.TurnRate = turnrate;
	return;
}

void Roomba::RoombaRestart()
{
	RoombaEnd();
	RR.end = false;
	RR.fd = 0;
	usleep(300000);
	RoombaStart();
}


int Roomba::GetDistance() { return RR.distance; }	


double Roomba::GetAngle() { return RR.angle; }



int Roomba::GetBattery() {	return RR.battery; }	

void Roomba::VacuumOn() { RR.vacuumOn = true; }
		
void Roomba::VacuumOff() { RR.vacuumOff = true; }

int Roomba::GetLeftBumper() { return RR.leftbump; }

int Roomba::GetRightBumper() { return RR.rightbump; }

int Roomba::GetRightWheel() { return RR.rightwheel; }
		
int Roomba::GetLeftWheel() { return RR.leftwheel; }

int Roomba::GetCasterWheel() { return RR.casterwheel; }







