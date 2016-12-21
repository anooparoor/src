
#define PI 3.14159265

struct UG01laser
{
	const char* Port;
   int fd;
   bool end;
   long data[682];
   double left, right, straight;
	bool Disp;

};



void *UG01Thread(void *L2)
{
	laser *LL = (laser *)L2;
	
	int range2[2] = { 44, 725 };
	int group = 1;
	char recv_buffer[1500];//3072
	bool good = false;
	int i = 0;
	int numread;
	int total = 0;
	int value;
	
	
	char message[14];
	sprintf(message, "GS0044072501\n\0");	
	char reply[24];
	sprintf(reply, "                       \0");
	char stopURG[4];
	sprintf(stopURG, "QT\n\0");
	
	
	
	printf( "URG connected on %s\n", LL->Port );
	
	
	while(LL->end == false)
	{		
		good = false;
		while(good == false)
		{
			write(LL->fd, message, 13);
			usleep(150000);	
			numread = read(LL->fd,reply,23);

			total = 0;
			int nodetotal = 0;
			int n = 0;
		
			while(total < 1409)
			{
				numread = read(LL->fd,recv_buffer,1500);		
				total = total + numread;
			}

			int k = 0;
			int skip = 64;
			for (int j = 0; j <= 681; j++)
			{
				LL->data[j] = 0;
				LL->data[j] = (recv_buffer[k]-48 << 6) | (recv_buffer[k+1]-48);
				k++;
				k++;
				if(k == skip){k++;k++;skip = skip + 66;}
			}
		
			LL->straight = LL->data[340];
	
			value = 5000;
			for(int i = 0; i < 340; i++)
			{
				if(LL->data[i] < value && LL->data[i] > 20)
					value = LL->data[i];
			}
			if(value == 5000) LL->right = 0;
			else LL->right = value;
			
			value = 5000;
			for(int i = 341; i < 682; i++)
			{
				if(LL->data[i] < value && LL->data[i] > 20)
					value = LL->data[i];
			}
			if(value == 5000) LL->left = 0;
			else LL->left = value;
		}
		
	}	
	printf("Closing connection to URG\n");
		
	//////////////////// Stops URG Laser //////////////////
	cout << "Stoping URG Laser\n";
	sprintf(reply, "                       \0");
	//sprintf(stopURG, "QT\n");
	cout << stopURG;
	while(reply[0] != 'Q')
	{
		usleep(100000);
		if(write(LL->fd, stopURG, 3) < 0)
		{
			perror("create_get_sensors():write():");
			return NULL;
		}
	
		numread = read(LL->fd,reply,23);
		cout << numread << endl;
		cout << reply;
		//fprintf(stdout, "%d\n", reply[5]);
	}
	cout << "URG Laser Stopped\n\n";
	//////////////////// Stops URG Laser //////////////////
	

	pthread_exit(NULL);
}	





/*
void *URGDisplayThread(void *L2)
{
	laser *LL = (laser *)L2;
	
	int map[500][500];//static int map[500][500];
	int locationX = 250;
	int locationY = 250;
	int MaxRange = 2880;//1824;
	int MaxPts = 240;//152;	//(MaxRange/12)
	
	int x, y;
	double laserAngle;
	double range;
	int distance;
	double radians;
	
	char* filename =  (char*)"URGImg.bmp";//argc >= 2 ? argv[1] :
	IplImage* img = 0; // pointer to an image 
	cvNamedWindow("Hokuyo URG", 0 ); // create a window  CV_WINDOW_AUTOSIZE

	cvStartWindowThread();

	while(LL->Disp == true)
	{
		for(int r = 0; r < 500; r++)
		{
			for(int c = 0; c < 500; c++)
			{ 
				map[r][c] = -1; 
			}
		}
		
		laserAngle = -30;	
		
		for(int i = 0; i <= 681; i++)
		{
			range = LL->data[i];
			distance = MaxPts;				
			radians = laserAngle * (PI / 180);
		
			if(laserAngle <= 210 && laserAngle >= -30)
			{
				if (range > MaxRange || range <= 20)
				{
					while (distance >= 0) 
					{	
						y = locationY - ((distance * sin(radians)) + .5);
						x = locationX + ((distance * cos(radians)) + .5);
						if(map[y][x] != 1)	map[y][x] = 0;  
						distance--;							
					}
				}
				if (range <= MaxRange && range > 20)
				{
					distance = (range / 12); 
					y = locationY - ((distance * sin(radians)) + .5);
					x = locationX + ((distance * cos(radians)) + .5);
					map[y][x] = 1;	
					map[y+1][x] = 1;	
					map[y-1][x] = 1;	
					map[y][x+1] = 1;	
					map[y][x-1] = 1;						
					distance--;						
					
					while (distance >= 0)
					{	
						y = locationY - ((distance * sin(radians)) + .5);
						x = locationX + ((distance * cos(radians)) + .5);			
						if(map[y][x] != 1)
							map[y][x] = 0;	
						distance--;						
					}
				}
			}								
			laserAngle += .3515625;
		}
		
		URGBMP(map);
		
		img = cvLoadImage(filename, 1); 	 
		cvShowImage("Hokuyo URG", img); 

	}

	cvReleaseImage(&img);
	cvDestroyWindow("Hokuyo URG");
	cvWaitKey(1);
	cvDestroyWindow("Hokuyo URG");
   cvWaitKey(1); 
   cvWaitKey(1); 
	pthread_exit(NULL);
}

	*/
	