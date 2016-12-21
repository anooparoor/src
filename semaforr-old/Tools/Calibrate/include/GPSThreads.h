
struct sGPS
{
	const char* Port;
   int fd;
   bool end;
	int Inorthmin;	//North minutes interger
	int Inorthsec;	//North seconds interger
	int Inorthdeg;	//North degrees interger
	int Iwestmin;	//West minutes interger
	int Iwestsec;	//West seconds interger
	int Iwestdeg;
	int northtotal;
	int westtotal;
};


bool GPSRead(char sResult[], int fd)
{
	int	iBytesRead = 0;
	int total = 0;
	char	sBuffer[60];
	
	
	while(total < 1)//43)//43
	{
		while(iBytesRead < 1)
		{
			iBytesRead = read(fd,sBuffer + total,43 - total);//43
		}
		total += iBytesRead;
	}
	for(int i = 0; i <= 43; i++)	sResult[i] = sBuffer[i];
	
	if(tcflush(fd, TCIFLUSH) < 0 )
  	{
    	perror("create_open():tcflush():");
    	close(fd);
    	fd = -1;
    	return(-1);
  	}
  	
	usleep(600000);
	return true;
	
}



bool Extract(char sResult[], sGPS* GG)
{
	if (sResult[0] == '$' && sResult[41] == 'W')	
	{
		string Snorthmin;
		string Snorthsec;
		string Snorthdeg;
		string Swestmin;
		string Swestsec;
		string Swestdeg;
		
				
		for (int i = 0; i < 2; i++)
		{
			Snorthdeg += sResult[18 + i];
			Swestdeg += sResult[31 + i];
		}
		GG->Inorthdeg = atoi(Snorthdeg.c_str());
		GG->Iwestdeg = atoi(Swestdeg.c_str());
		
		for (int i = 0; i < 2; i++)
		{
			Snorthmin += sResult[20 + i];
			Swestmin += sResult[33 + i];
		}
		GG->Inorthmin = atoi(Snorthmin.c_str());
		GG->Iwestmin = atoi(Swestmin.c_str());

		for (int i = 0; i < 4; i++)
		{
			Snorthsec += sResult[23 + i];
			Swestsec += sResult[36 + i];
		}
		GG->Inorthsec = atoi(Snorthsec.c_str());
		GG->Iwestsec = atoi(Swestsec.c_str());
		GG->northtotal = ((GG->Inorthdeg * 1000000) + (GG->Inorthmin * 10000) + GG->Inorthsec);	 
	 	GG->westtotal = ((GG->Iwestdeg * 1000000) + (GG->Iwestmin * 10000) + GG->Iwestsec);
		return true;
	}
	return false;
}



void *GPSThread(void *G2)
{
	sGPS *GG = (sGPS *)G2;
	char sResult[60];
	
	while(GG->end == false)
	{
		GPSRead(sResult, GG->fd);
		//cout << "GPS Code:\n" << sResult << endl;
		Extract(sResult, GG);
/*		
		if(local.Extract(sResult, GG))
		{
			runs++;
			system("clear");
			cout << "runs = "<< runs << endl;
			cout << "GPS Code:\n" << sResult << endl;
			cout << "GPS Courdinates:\n";
			local.print();
		}
		
*/
    }
    printf("Closing connection to GPS\n");
	 pthread_exit(NULL);
}






