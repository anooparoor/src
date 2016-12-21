#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H


#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <sys/time.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <string>


using namespace std;

class BlackfinConfig 
{
	public:
		BlackfinConfig();
	  	BlackfinConfig(const char* rc);
	
	   const char* robotconf;
	   string plugin;
		string IP;
		
		int left_speed;
		int right_speed;
		int cm_per_sec;
		int degrees_per_sec;
	
		int cm0_10;
		int cm11_20;
		int cm21_30;
		int cm31_40;
		int cm41_50;
		int cm51_60;
		int cm61_plus;
		
		double dg0_20;
		double dg21_40;
		double dg41_60;
		double dg61_80;
		double dg81_100;
		double dg101_120;
		double dg121_140;
		double dg141_160;
		double dg161_180;
		double dg181_360;
	  
		bool read();
		bool read(const char* rc); 
		bool write();
		bool write(const char* rc);

};

#endif /* CONFIGMANAGER_H */
