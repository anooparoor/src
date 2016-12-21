/*
 * Controller.cpp
 *
 */

#include "ConfigManager.h"

using namespace std;


BlackfinConfig::BlackfinConfig(){}

BlackfinConfig::BlackfinConfig(const char* rc)
{
	robotconf = rc;
	read(robotconf);
}

bool BlackfinConfig::read()
{
	return read(robotconf);
}

bool BlackfinConfig::read(const char* rc)
{
	cout << "Reading Blackfin Config File: " << rc << endl;
	ifstream rFile(rc);
	if(!rFile)
   {
     	cout << "unable to open file: " << rc << endl << endl; 
   	exit (1); 
   }
	
	string cmd;
	while( !rFile.eof() ) 
  	{
		cmd = ""; 
    	rFile >> cmd ;
    	
    	if (! (cmd[0] == '#'  ||  cmd == "" ))
	   {   
	      if ( cmd == "ip" )
	      {
	      	rFile.get();
			  	getline(rFile, IP);
	      }
	      if ( cmd == "left_speed" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> left_speed;
	      }
	      if ( cmd == "right_speed" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> right_speed;
	      }
	      if ( cmd == "cm_per_sec" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm_per_sec;
	      }
	      if ( cmd == "degrees_per_sec" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> degrees_per_sec;
	      }
	      if ( cmd == "cm0_10" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm0_10;
	      }
	      if ( cmd == "cm11_20" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm11_20;
	      }
	      if ( cmd == "cm21_30" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm21_30;
	      }
	      if ( cmd == "cm31_40" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm31_40;
	      }
	      if ( cmd == "cm41_50" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm41_50;
	      }
	      if ( cmd == "cm51_60" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm51_60;
	      }
	      if ( cmd == "cm61_plus" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> cm61_plus;
	      }
	      
	      if ( cmd == "dg0_20" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg0_20;
	      }
	      if ( cmd == "dg21_40" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg21_40;
	      }
	      if ( cmd == "dg41_60" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg41_60;
	      }
	      if ( cmd == "dg61_80" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg61_80;
	      }
	      if ( cmd == "dg81_100" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg81_100;
	      }
	      if ( cmd == "dg101_120" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg101_120;
	      }
	      if ( cmd == "dg121_140" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg121_140;
	      }
	      if ( cmd == "dg141_160" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg141_160;
	      }
	      if ( cmd == "dg161_180" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg161_180;
	      }
	      if ( cmd == "dg181_360" )
	      {
	      	rFile.get();rFile.get();
			  	rFile >> dg181_360;
	      }
	      if ( cmd == "plugin" )
	      {
	      	rFile.get();
			  	getline(rFile, plugin);
	      }
		}	
	}
	rFile.close();
	cout << "File Read in Successfully\n\n";
	return true;
}

bool BlackfinConfig::write()
{
	return write(robotconf);
}

bool BlackfinConfig::write(const char* rc)
{
	//cout << "Writing To Config File\n";
	ofstream myfile;
  	myfile.open (rc);
  	/*
  	if(!rFile)
   {
     	cout << "unable to open file: " << rc << endl; 
   	return false; 
   }
   */
  	myfile << "driver\n(\n\tname \"Blackfin\"\n\tplugin " << plugin << "\n\t";
  	myfile << "provides [\"position2d:0\"]\n\tip " << IP << "\n\n";
  	myfile << "\t#Calibration Specs\n\tleft_speed [" << left_speed << "]\n\tright_speed [";
	myfile << right_speed << "]\n\tcm_per_sec [" << cm_per_sec; 
	myfile << "]\n\tdegrees_per_sec [" << degrees_per_sec << "]\n\n\t#Centimeters Per Second\n\t";
	myfile << "cm0_10 [" << cm0_10 << "]\n\tcm11_20 [" << cm11_20 << "]\n\tcm21_30 [";
  	myfile << cm21_30 << "]\n\tcm31_40 [" << cm31_40 << "]\n\tcm41_50 [" << cm41_50;
  	myfile << "]\n\tcm51_60 [" << cm51_60 << "]\n\tcm61_plus [" << cm61_plus;
  	myfile << "]\n\n\t#Degrees Per Second\n\tdg0_20 [" << dg0_20 << "]\n\tdg21_40 [";
  	myfile << dg21_40 << "]\n\tdg41_60 [" << dg41_60 << "]\n\tdg61_80 [" << dg61_80;
  	myfile << "]\n\tdg81_100 [" << dg81_100 << "]\n\tdg101_120 [" << dg101_120;
  	myfile << "]\n\tdg121_140 [" << dg121_140 << "]\n\tdg141_160 [" << dg141_160;
  	myfile << "]\n\tdg161_180 [" << dg161_180 << "]\n\tdg181_360 [" << dg181_360 << "]\n)";
  	myfile.close();
  	//cout << "Writing Complete\n\n";
  	return true;
}





