

#include "BlackfinThreads.h"


class Blackfin
{
	private:
		surv SS;
		pthread_t threads[5]; 
		struct sockaddr_in echoserver;
		unsigned int echolen;
	
	public:
		Blackfin(const char* IP2, const char* Image2);//Blackfin SRV("192.168.1.10","image.jpg");
		void BlackStart();					//SRV.BlackStart();
		void BlackRestart();
		void BlackEnd();						//SRV.BlackEnd();
		void DisplayOn();						//SRV.DisplayOn();
		void DisplayOff();					//SRV.DisplayOff();
		void RecordOn(const char* video);//SRV.RecordOn("video.avi");      *Only Works if Display is On*
		void RecordOff();						//SRV.RecordOff();
		void LasersOn();						//SRV.LasersOn();
		void LasersOff();						//SRV.LasersOff();
		char Battery();						//char BatteryLife;			l = low battery   b = Battery OK
							  						//BatteryLife = SRV.Battery();
		IplImage* GetImg();					//IplImage* image; 
													//image = SRV.GetImg();
		void SetMotors(double speed, double turnrate);//double speed = 50;			speed: (-100 - 100)
																	 //double turnrate = 1.5;		turnrate: (-3.3 - 3.3)
};																	 //SRV.SetMotors(speed, turnrate);



//////////////////////////////////////// Blackfin() /////////////////////////////////

Blackfin::Blackfin(const char* IP2, const char* Image2)
{
	printf("Initializing SRV-1 Blackfin\n");
	SS.img = 0;
	SS.Image = Image2;
	SS.Port = 10001;
	SS.IP = IP2;
	SS.Speed = 0;
	SS.TurnRate = 0;
	SS.cameraOn = false;
	SS.controlOn = true;
	SS.displayOn = false;
	SS.record = false;
	SS.pauseOut = false;
	SS.setlaser = false;
	SS.unsetlaser = false;
	SS.laserOn = false;
	SS.getRange = false;
	SS.showdisp = false;
	SS.end = false;
	SS.getimg = false;
	SS.getBattery = false;
	SS.battery = ' ';
}

//////////////////////////////////////// BlackStart() /////////////////////////////////

void Blackfin::BlackStart()
{
	printf("Connecting to SRV-1 Blackfin on %s\n", SS.IP);
	if ((SS.sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
   {
     Die("Failed to create socket");
	}
		
	memset(&echoserver, 0, sizeof(echoserver));   
   echoserver.sin_family = AF_INET;              
   echoserver.sin_addr.s_addr = inet_addr(SS.IP); 
   echoserver.sin_port = htons(SS.Port);      

	if (connect(SS.sock, (struct sockaddr *) &echoserver, sizeof(echoserver)) < 0) 
   {
   	printf("Make sure the IP adresses and ports match\n");
   	Die("Failed to connect with server");
   }
 		

	int t=0;
	int rc = pthread_create(&threads[0], NULL, MainThread, (void *) &SS ); //(void *) t

	t=1;		
	rc = pthread_create(&threads[0], NULL, CameraThread, (void *) &SS);

	usleep(200000);

	return;
}

//////////////////////////////////////// BlackEnd() /////////////////////////////////

void Blackfin::BlackEnd()
{
	SS.showdisp = false;
	SS.end = true;
	sleep(2);
	close(SS.sock);
}

//////////////////////////////////////// DisplayOn() /////////////////////////////////

void Blackfin::DisplayOn()
{
	if(SS.showdisp == false)
	{
		int t=2;
		int rc = pthread_create(&threads[0], NULL, DisplayThread, (void *) &SS);	
	}
	SS.showdisp = true;
	return;
}

//////////////////////////////////////// DisplayOff() /////////////////////////////////

void Blackfin::DisplayOff()
{
	SS.showdisp = false;
	return;
}

//////////////////////////////////////// RecordOn() /////////////////////////////////

void Blackfin::RecordOn(const char* video)
{
	if(SS.showdisp == true)
	{	
		sleep(1);
		while(SS.displayOn == false){}
		double fps = 9;
		CvSize size = cvSize(SS.img->width, SS.img->height);
		SS.writer = cvCreateVideoWriter(video,CV_FOURCC('D', 'I', 'V', 'X'),fps,size);
		SS.record = true;
	}
	else 	printf("Display Must Be Turned On First/n");
}

//////////////////////////////////////// RecordOff() /////////////////////////////////

void Blackfin::RecordOff()
{
	SS.record = false;
}

//////////////////////////////////////// LasersOn() /////////////////////////////////

void Blackfin::LasersOn()
{
	SS.setlaser = true;
}

//////////////////////////////////////// LasersOff() /////////////////////////////////

void Blackfin::LasersOff()
{
	SS.unsetlaser = true;
}

//////////////////////////////////////// Battery() /////////////////////////////////

char Blackfin::Battery()
{
	SS.getBattery = true;
	while(SS.getBattery == true){}
	return SS.battery;
}
	
//////////////////////////////////////// GetImg() /////////////////////////////////
	
IplImage* Blackfin::GetImg()
{
	SS.getimg = true;
	while(SS.getimg == true){}
	return SS.img;
}

//////////////////////////////////////// SetMotors() /////////////////////////////////

void Blackfin::SetMotors(double speed, double turnrate)
{
	SS.Speed = speed;
	SS.TurnRate = turnrate;	
}


void Blackfin::BlackRestart()
{
	BlackEnd();	
	SS.img = 0;
	SS.Port = 10001;
	SS.Speed = 0;
	SS.TurnRate = 0;
	SS.cameraOn = false;
	SS.controlOn = true;
	SS.displayOn = false;
	SS.record = false;
	SS.pauseOut = false;
	SS.setlaser = false;
	SS.unsetlaser = false;
	SS.laserOn = false;
	SS.getRange = false;
	SS.showdisp = false;
	SS.end = false;
	SS.getimg = false;
	SS.getBattery = false;
	SS.battery = ' ';
	usleep(300000);
	BlackStart();	
	usleep(300000);

}
