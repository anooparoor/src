

struct surv
{
	
	IplImage* img; 
	const char* Image;
	CvVideoWriter* writer;
	int Port;
	const char* IP;
	int sock;
	double Speed;
	double TurnRate;
	bool cameraOn;
	bool controlOn;
	bool displayOn;
	bool record;
	bool pauseOut;
	bool setlaser;
	bool unsetlaser;
	bool laserOn;
	bool getRange;
	bool showdisp;
	bool end;
	bool getimg;
	bool getBattery;
	char battery;	

};


void *CameraThread(void *SR)
{
	surv *SS = (surv *)SR;
	char test[1];
   test[0] = 'I';
   
   int BUFFSIZE=10000;
   char buffer2[BUFFSIZE];
   
   int bufPeakSize = 10;
   char bufPeak[bufPeakSize];
   char* filename = (char*)SS->Image;////////////
   
   int received = 0;
	int total = 0;
	int frame_size = 0;
	
	ofstream outfile;

	while(SS->end == false)
	{
		if(SS->cameraOn == true)
		{
			SS->displayOn = false;
			received = 0;
			total = 0;
			send(SS->sock, test, 1, 4);
			received = recv(SS->sock, bufPeak, bufPeakSize, 0);
			int s0 = (unsigned char) bufPeak[6];
		   int s1 = (unsigned char) bufPeak[7];
		   int s2 = (unsigned char) bufPeak[8];
		   int s3 = (unsigned char) bufPeak[9];
			frame_size = s0 + (s1 * 256) + (s2 * 256 * 256) + (s3 * 256 * 256 * 256);
		  	received = recv(SS->sock, buffer2, BUFFSIZE, 0);
		   total = total + received;
			frame_size = frame_size;
			while(SS->pauseOut == true){}
			outfile.open(SS->Image, ofstream::binary | ofstream::trunc);
			outfile.write ((char*)buffer2,received);
			while(total < frame_size) 
			{
				received = recv(SS->sock, buffer2, BUFFSIZE, 0);
		     	outfile.write ((char*)buffer2,received);
		      total = total + received;
			}
			
			if(SS->getimg == true)	SS->getimg = false;
			outfile.close();	
			SS->img = cvLoadImage(filename, 1);	///////////////////////
			if(SS->showdisp==true) SS->displayOn = true;				
			SS->cameraOn = false;
			SS->controlOn = true;
		}	
	}
	pthread_exit(NULL);

}



void *DisplayThread(void *SR)
{
	cvStartWindowThread();
	surv *SS = (surv *)SR;
	char* filename = (char*)SS->Image;
	cvNamedWindow(SS->Image, 0 ); 

	//SS->img = cvLoadImage(filename, 1);

	while(SS->showdisp == true)
	{
		if(SS->displayOn == true)
		{
			SS->pauseOut = true;
			//SS->img = cvLoadImage(filename, 1);	
			SS->displayOn = false;
			SS->pauseOut = false; 
			cvShowImage(SS->Image, SS->img);
			if(SS->record == true)
				cvWriteFrame( SS->writer, SS->img );
		}		
	}	
	cvReleaseImage( &SS->img );
	cvDestroyWindow( SS->Image );
	cvWaitKey(1);
	cvDestroyWindow( SS->Image );
   cvWaitKey(1); 
   cvWaitKey(1); 
	pthread_exit(NULL);
}







void *MainThread(void *SR)
{
	surv *SS = (surv *)SR;
 	
 	printf( "Surveyor Blackfin connected on %s\n", SS->IP );
	
	double speed; 
  	double angle;
	double X, Y;
	int commandSize = 4;
	char command[4];
	char end[2];
	int laserbufsize = 50;
	char laserbuf[laserbufsize];	
	int shortbufsize = 1;
	int irange = 0;
	unsigned char crange;
	char srange[6];
	char shortbuf[1];
   int BUFFSIZE2 = 4;
   int BUFFSIZE = 600;
   int runtime = 0;
   char buffer[BUFFSIZE];
   int received = 0;
   int rec = 0;
	int bytes = 0;
	char testbuff[100];
	
	shortbuf[0] = 'V';
	send(SS->sock, shortbuf, 1, 4);
	
	while (received < 1) 
	{	
		if((bytes = recv(SS->sock, testbuff, 100, 0)) < 1) //-1
		{
			Die("Failed to receive bytes from server//");
		}
  		received += bytes;
  		testbuff[bytes] = '\0';    
	}
   
   fprintf(stdout, "\n\nsrv1_init(): successful init. HW\n\n %s \n\n", testbuff);

	while(SS->end == false)
	{				
		if(SS->controlOn == true)
		{
			if(SS->setlaser == true && SS->laserOn == false)
			{
				received = 0;
				shortbuf[0] = 'l';
				send(SS->sock, shortbuf, shortbufsize, 4);
				while (received < 1) 
			   {	
					if ((received = recv(SS->sock, buffer, BUFFSIZE - 1, 0)) < 1) //-1
					{
						Die("Failed to receive bytes from server");
					}
			     buffer[received] = '\0';    
			   }
			   printf( "Lasers On %s\n\n", buffer);
			   SS->setlaser = false;
			   SS->unsetlaser = false;
			   SS->laserOn = true;
			}
			if(SS->getRange == true)
			{
				received = 0;
				shortbuf[0] = 'R';
				send(SS->sock, shortbuf, shortbufsize, 4);
				while (received < 1) 
			   {	
					received = recv(SS->sock, laserbuf, laserbufsize, 0);
				}
				buffer[received] = '\0';
				int j = 0;
				for(int i = 14; i < received - 2; i++)
				{
					srange[j] = laserbuf[i];
					j++;	
				}
				srange[j] = '\0';
			   printf( "Range is: %s\n", srange);	
			   SS->getRange = false;	
			   SS->laserOn = false;	
			}
			if(SS->unsetlaser == true && SS->laserOn == true)
			{
				SS->setlaser = false;
				SS->unsetlaser = false;
			   SS->laserOn = false;
				received = 0;
				shortbuf[0] = 'L';
				send(SS->sock, shortbuf, shortbufsize, 4);
				while (received < 1) 
			   {	
					if ((received = recv(SS->sock, buffer, BUFFSIZE - 1, 0)) < 1) //-1
					{
						Die("Failed to receive bytes from server");
					}
			     buffer[received] = '\0';    
			   }
			   printf( "Lasers Off %s\n\n", buffer);
			}
			if(SS->getBattery == true)
			{
				received = 0;
				shortbuf[0] = 'D';
				send(SS->sock, shortbuf, shortbufsize, 4);
				while (received < 1) 
			   {	
					received = recv(SS->sock, laserbuf, laserbufsize, 0);
				}
				buffer[received] = '\0';
				SS->battery = laserbuf[6];
			   SS->getBattery = false;	
			}
			received = 0;
			X = SS->TurnRate * -1;
			X = X / 10000;
			X = X / 2;
			Y = SS->Speed * -1;
			Y = Y / 1000;
			Y = Y * 2.5;
			Y = Y / 100;
			
			if(Y < -.1)
			{
				X = X * -1;
			}
			
			speed = Y;

	   	speed = speed * 100;
	   	angle = X;
	   	
	   	speed = SS->Speed;
	   	angle = SS->TurnRate;
	   	
	   	
			if(speed < 10 && speed > -10) speed = 0;	
	   
	   	int leftspeed = speed;
	   	int rightspeed = speed;
	   
	   
			if(angle < 0) 
		   {
		   	angle = angle * -1;
		   	angle = angle * 32;
		   	if(angle > 200) angle = 200;
		   	leftspeed += angle;
		   	rightspeed -= angle;
		   }
		   else if(angle > 0)
		   {
		   	angle = angle * 32;
		   	if(angle > 200) angle = 200;
		   	leftspeed -= angle;
		   	rightspeed += angle;
		   }
	   
		   
		   if(rightspeed > 100) rightspeed = 100;
		   if(leftspeed > 100) leftspeed = 100;
		   if(rightspeed < -100) rightspeed = -100;
		   if(leftspeed < -100) leftspeed = -100;
		   
			signed char ll = leftspeed;
			signed char rr = rightspeed;
			signed char tt = 00;
		
			command[0] = 'M';
		   command[1] = ll;
		   command[2] = rr;
		   command[3] = tt;
			send(SS->sock, command, commandSize, 4);
	
		   while (received < 1) 
		   {	
				if ((received = recv(SS->sock, buffer, BUFFSIZE - 1, 0)) < 1) //-1
				{
					Die("Failed to receive bytes from server");
				}
		     received += rec;
		     buffer[received] = '\0';    
		   }		  
		   
			if(SS->end == false)
			{
				SS->cameraOn = true;
			}
			SS->controlOn = false;
			
		}	
	}
	
	printf("Closing connection to Surveyor\n");
	received = 0;
	end[0] = '$';
   end[1] = '!';

/*
	send(SS->sock, end, 2, 4);
	usleep(200000);
	send(SS->sock, end, 2, 4);
	usleep(200000);
	send(SS->sock, end, 2, 4);
	usleep(200000);
	send(SS->sock, end, 2, 4);
	usleep(200000);
*/
	pthread_exit(NULL);
}


