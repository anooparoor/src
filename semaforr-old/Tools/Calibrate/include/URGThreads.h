
#define PI 3.14159265

struct laser
{
	const char* Port;
   int fd;
   bool end;
   long data[682];
   double left, right, straight;
	bool Disp;

};

static unsigned char encode6bit(char ch) 
{
	return 0x3f & (ch - 0x30);
}


void *URGThread(void *L2)
{
	laser *LL = (laser *)L2;
	char message[10];
	int range2[2] = { 44, 725 };
	int group = 1;
	char recv_buffer[1500];//3072
	char reply[12];
	bool good = false;
	int i = 0;
	int numread;
	int total = 0;
	int value;
	
	printf( "URG connected on %s\n", LL->Port );
	sprintf(message, "G%03d%03d%02d\r", range2[0], range2[1], group);
	
	while(LL->end == false)
	{		
		good = false;
		while(good == false)
		{
			usleep(100000);
			write(LL->fd, message, 10);

			usleep(10000);
			total = 0;
	
			while(total < 12)
			{
				numread = read(LL->fd,reply + total,12);
				if(numread > 0) total =+ numread;
			}
	
			if(reply[0] == 'G' && reply[1] == '0') good = true;
			total = 0;
			
			while(total < 200)
			{
				numread = read(LL->fd,recv_buffer + total,1500);
				if(numread != -1)
				if(numread > 0)	total =+ numread;
			}
			if(total != 1387) good = false;
		}

		i = 0;
		int actual_index = (i << 1) + (i >> 5);
	
		for (int j = 0; j <= 681; j++)
		{
			LL->data[j] = (encode6bit(recv_buffer[actual_index]) << 6) | encode6bit(recv_buffer[actual_index +1]);
			actual_index++;
			actual_index++;
			if(recv_buffer[actual_index] == 10) actual_index++;
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
	printf("Closing connection to URG\n");
	pthread_exit(NULL);
}	





#pragma pack(push,2)
struct ITMAPFILEHEADER 
{
	unsigned short    bfType;
	unsigned long   	bfSize;
	unsigned short    bfReserved1;
	unsigned short    bfReserved2;
	unsigned long   	bfOffBits;
};
#pragma pack(pop)

struct ITMAPINFOHEADER
{
	unsigned long     biSize;
	long       			biWidth;
	long       			biHeight;
	unsigned short    biPlanes;
	unsigned short    biBitCount;
	unsigned long     biCompression;
	unsigned long     biSizeImage;
	long       			biXPelsPerMeter;
	long      			biYPelsPerMeter;
	unsigned long     biClrUsed;
	unsigned long		biClrImportant;
};

struct RGB
{
	unsigned char    rgbtBlue;
	unsigned char    rgbtGreen;
	unsigned char    rgbtRed;
};

void URGBMP(int map[500][500]) 
{  
	ITMAPFILEHEADER fileHeader; 
	ITMAPINFOHEADER fileInfo; 
	unsigned long write = 0; 
	static RGB pic[500][500];

	ofstream outfile ("URGImg.bmp",ofstream::binary);//Remove

	fileHeader.bfType = 19778;       
	fileHeader.bfSize = sizeof(fileHeader.bfOffBits) + sizeof(RGB);
	fileHeader.bfReserved1 = 0;                                        
	fileHeader.bfReserved2 = 0; 
	fileHeader.bfOffBits = sizeof(ITMAPFILEHEADER)+sizeof(ITMAPINFOHEADER);                   

	//cout << sizeof(ITMAPFILEHEADER) << endl;

	fileInfo.biSize = sizeof(ITMAPINFOHEADER); 
	fileInfo.biWidth = 500; 
	fileInfo.biHeight = 500; 
	fileInfo.biPlanes = 1; 
	fileInfo.biBitCount = 24; 
	fileInfo.biCompression = 0L;   //BI_RGB; 
	fileInfo.biSizeImage = 500 * 500 * (24/8); //512
	fileInfo.biXPelsPerMeter = 2400; 
	fileInfo.biYPelsPerMeter = 2400; 
	fileInfo.biClrImportant = 0; 
	fileInfo.biClrUsed = 0; 


	outfile.write ((char*)&fileHeader,sizeof(fileHeader));//remove
	outfile.write ((char*)&fileInfo,sizeof(fileInfo));//remove



	int i = (500-1);
	for(int r = 0; r < 500; r++)
	{
		for(int c = 0; c < 500; c++)
		{
			switch (map[r][c])
			{
			case -1: 
				pic[i][c].rgbtBlue = 255; 
				pic[i][c].rgbtGreen = 255; 
				pic[i][c].rgbtRed = 255;
				break;
			case 1: 
				pic[i][c].rgbtBlue = 0; 
				pic[i][c].rgbtGreen = 0; 
				pic[i][c].rgbtRed = 0;
				break;
			case 0: 
				pic[i][c].rgbtBlue = 0; 
				pic[i][c].rgbtGreen = 0; 
				pic[i][c].rgbtRed = 255;
				break;
			case 2: 
				pic[i][c].rgbtBlue = 0; 
				pic[i][c].rgbtGreen = 0; 
				pic[i][c].rgbtRed = 255;
				break;
			case 8:
				pic[i][c].rgbtBlue = 0; 
				pic[i][c].rgbtGreen = 255; 
				pic[i][c].rgbtRed = 0;
				break;
			case 9:
				pic[i][c].rgbtBlue = 0; 
				pic[i][c].rgbtGreen = 0; 
				pic[i][c].rgbtRed = 255;
				break;
			
			}
		}
		i--;
	}

	outfile.write ((char*)pic,fileInfo.biSizeImage);//remove


	outfile.close();//remove
	return;
} 




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

	
	