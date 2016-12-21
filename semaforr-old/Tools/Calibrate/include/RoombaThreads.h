


struct Room
{
	const char* Port;
	double Speed;
	double TurnRate;
	double distance;
	double angle;
	int iangle;
	int idistance;
	int leftbump;
	int rightbump;
	int leftwheel;
	int rightwheel;
	int casterwheel;
	int battery;
	int flags;
   int fd;
   bool end;
   bool vacuumOn;
   bool vacuumOff;
   bool vacuumRun;


};


void *RoombaThread(void *R2)
{
	Room *RR = (Room *)R2;

	printf( "Roomba connected on %s\n", RR->Port );	

	int16_t tv_mm, rad_mm;
	unsigned char cmd5buf[5];
	unsigned char cmd2buf[2];
	unsigned char databuf[10];
	unsigned char distbuf[6];
	unsigned char cmdbuf[2];
	int irad = 0;
	float frad = 0;
	int ivel = 0;
	float fvel = 0;
	int test = 0;
  	int retval;
  	int numread = 0;
  	int total = 0;
	int bat = 0;
	int iangle = 0;
	int idistance = 0;
	double dangle = 0;
	
	
	while(RR->end == false)
	{
		//system("clear");

		
///////////////////////////////// Motor Commands /////////////////////////////////////
		frad = RR->TurnRate;
		fvel = RR->Speed;
		
		fvel = fvel * 500;
		ivel = (int)fvel;
		if(ivel < 10 && ivel > -10) ivel = 0;
		
		frad = frad * 2000;
		if(ivel != 0)
		{
			if(frad > 100)
			{
				frad = 2000 - frad;
				irad = (int)frad;
			}
			else if(frad < -100)
			{
				frad = -2000 - frad;
				irad = (int)frad;
			}
			else irad = 32768;
		}
		if(ivel == 0 && frad > 0) {irad = 1; ivel = frad / 4;}
		if(ivel == 0 && frad < 0) {irad = -1; ivel = frad/-4;}
		
		//printf( "Velocity: %d \n", ivel);
		//printf( "Radius: %d \n", irad);
		
		cmd5buf[0] = 137;
		cmd5buf[1] = ivel >> 8;
		cmd5buf[2] = (unsigned char) ivel;
		cmd5buf[3] = irad >> 8;
		cmd5buf[4] = (unsigned char) irad;
		
      write(RR->fd, cmd5buf, 5);

  		usleep(30000);  
///////////////////////////////// Motor Commands /////////////////////////////////////	
  		
  		
  		
///////////////////////////////// Vacuum Commands /////////////////////////////////////
		if(RR->vacuumOn == true && RR->vacuumRun == false)
		{
			cmdbuf[0] = 138;
			cmdbuf[1] = 7;
			
			write(RR->fd, cmdbuf, 2);

			RR->vacuumOn = false;
			RR->vacuumRun = true;
			usleep(30000);  	  
		}

		if(RR->vacuumOff == true && RR->vacuumRun == true)
		{
			cmdbuf[0] = 138;
			cmdbuf[1] = 0;
			
			write(RR->fd, cmdbuf, 2);

			RR->vacuumOff = false;	 
			RR->vacuumRun = false; 
			usleep(30000);  
		}
///////////////////////////////// Vacuum Commands /////////////////////////////////////



///////////////////////////////// Sensor Commands /////////////////////////////////////
		total = 0;
		
		cmdbuf[0] = 142;
	  	cmdbuf[1] = 1;
	
	  	write(RR->fd, cmdbuf, 2);
	
		usleep(30000);//30000
		total = 0;
		while(total < 2)
		{
			numread = read(RR->fd,databuf + total,10);
			if(numread > 0)	total =+ numread;
		}
		
		//printf( "total: %d\n\n",total );
		//printf( "Message: %d ", databuf[0]);
		//for(int i = 1; i < 10; i++)
		//{
		//	printf( "%d ", databuf[i]);
		//}
	
		//printf( "\n\n");
	
		test = databuf[0] << 8;
		if(test == 256) RR->rightbump = 1;
		else RR->rightbump = 0;
		test = databuf[0] << 7;
		if(test == 256) RR->leftbump = 1;
		else RR->leftbump = 0;
		test = databuf[0] << 6;
		if(test == 256) RR->rightwheel = 1;
		else RR->rightwheel = 0;
		test = databuf[0] << 5;
		if(test == 256) RR->leftwheel = 1;
		else RR->leftwheel = 0;
		test = databuf[0] << 4;
		if(test == 256) RR->casterwheel = 1;
		else RR->casterwheel = 0;

		usleep(30000);//30000
///////////////////////////////// Sensor Commands /////////////////////////////////////


///////////////////////////////// Distance/Angle /////////////////////////////////////
		total = 0;
		iangle = 0;
		idistance = 0;	
		dangle = 0;
		
		cmdbuf[0] = 142;
	  	cmdbuf[1] = 2;
	
	  	write(RR->fd, cmdbuf, 2);
	
		usleep(30000);//30000
		total = 0;

		while(total < 2)
		{
			numread = read(RR->fd,distbuf + total,6);
			if(numread > 0)	total =+ numread;
		}
		
		//printf("%d\n", total);
		
		idistance = (int)distbuf[2];
		idistance = idistance << 8;
		idistance = idistance + (signed int)distbuf[3];		
		RR->distance = RR->distance + idistance;
		iangle = (int)distbuf[4];
		iangle = iangle << 8;
		iangle = iangle + (signed int)distbuf[5];		
		//printf("Distance %d\n", databuf[3] );
		//printf("angle: %d\n", databuf[4]);
		//printf("%s\n", distbuf);
		dangle = (360 * (double)iangle)/(258 * 3.14159265);
		RR->angle = RR->angle + dangle;
		if(RR->angle > 360) RR->angle = RR->angle - 360;
		if(RR->angle < -360) RR->angle = RR->angle + 360;
		
		usleep(30000);//30000
///////////////////////////////// Distance/Angle /////////////////////////////////////

/*
///////////////////////////////// Battery test /////////////////////////////////////
		total = 0;
		bat = 0;		
		
		cmdbuf[0] = 142;
	  	cmdbuf[1] = 3;
	
	  	write(RR->fd, cmdbuf, 2);
	
		usleep(30000);//30000
		total = 0;

		while(total < 2)
		{
			numread = read(RR->fd,databuf + total,10);
			if(numread > 0)	total =+ numread;
		}
		
		bat = (int)databuf[6];
		bat = bat << 8;
		bat = bat + (int)databuf[7];
		RR->battery = bat;
	
		usleep(30000);//30000
///////////////////////////////// Battery Test /////////////////////////////////////

*/
	}
	
	printf("Closing connection to Roomba\n");

	pthread_exit(NULL);
	
}