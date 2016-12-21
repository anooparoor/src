/*
 * Controller.cpp
 *
 */
#include <pthread.h>
#include "Controller.h"
#include <iostream> 
#include <fstream>
#include "TextColor.h"
#include <locale>
using namespace std;

#define CTRL_DEBUG false 

bool calibrating = false;

void *ReadThread(void *SR)
{
	PlayerClient *pCli = (PlayerClient *)SR;
	cout << "Starting Read Thread";
	while(1)
	{
		if(calibrating == false)
		{
			pCli->Read();
			sleep(1);
		}
	}
	
	pthread_exit(NULL);	
}

void *UpdateThread(void *SR)
{
	CommunicationManager *cMan = (CommunicationManager *)SR;
	
	while(1)
	{
		if(calibrating == false)
		{
			cMan->Update();
			usleep(1000);
		}
	}
	pthread_exit(NULL);	
}

Controller::Controller(string pcH, int pcP, string csH, int csP, string lab, string ty) {
	cout << "Setting Up Controller\n";
  try {
    pCli = new PlayerClient(pcH, pcP); 

    mot = new Motion(); 
    mot->setPosition2dProxy(pCli); 

    cMan = new CommunicationManager(*pCli, mot, lab, ty);
    
    if (!cMan->Connect(csH, csP)) {
      cerr << "Failed to establish a connection to the Central Server.\n"
	   << "Central Server hostname: " << csH << "\n"
	   << "Central Server port: " << csP << endl;
      exit(1);
    }
  }
  catch (PlayerError){
    cerr << "Failed to establish a connection to the Player Server.\n"
	 << "Robot: " << lab << ", type:" << ty << "\n"
	 << "Player Server hostname: " << pcH << "\n"
	 << "Player Server port: " << pcP << endl;
    exit(1);
  }
 
  ofname = lab + "-samples.dat";
  
  cout << "Controller Started Successfully\n\n";
  
  
   pthread_t threads[5]; 
	pthread_create(&threads[0], NULL, ReadThread, (void *) pCli ); //(void *) t
	pthread_create(&threads[0], NULL, UpdateThread, (void *) cMan ); //(void *) t
	
	  
}

void Controller::sampleLinearMotion() {
  // open file to write
  ofstream outfile(ofname.c_str(), ios::app); 
  if ( !outfile ) {
    cerr << "Could not open the file: " << ofname << endl;
    exit(1);    
  }

  for ( unsigned int repeat = 0 ; repeat < 100; repeat++ ){
    for ( int deltaX = 10; deltaX < 60 ; deltaX+=10) {
      // first move forward
      Position p(deltaX, 0, 0); 
      move(p); 
      // write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
      outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
	      << startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
	      << endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;
      sleep(1); 

      // move back twice
      p.setX(-1 * deltaX);
      for ( int i = 0; i < 2; i++ ) {
	move(p); 
	// write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
	outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
		<< startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
		<< endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;
	sleep(1); 
      }
      
      // move forward again
      p.setX(deltaX);
      move(p); 
      // write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
      outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
	      << startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
	      << endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;
      sleep(1); 
    }
  }

  // close file
  outfile.close();
}

void Controller::sampleRotationalMotion() {
  // open file to write
  ofstream outfile(ofname.c_str(), ios::app); 
  if ( !outfile ) {
    cerr << "Could not open the file: " << ofname << endl;
    exit(1);    
  }

  int deltaTheta[] = {8, 22, 37, 52, 75, 90, 125, 179};
  for ( unsigned int repeat = 0 ; repeat < 100; repeat++ ){
    for ( unsigned int i = 0; i < 8; i++) {
      Position p(0,0,Utils::toRadians(deltaTheta[i]));
      move(p); 
      // write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
      outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
	      << startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
	      << endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;
      sleep(1); 

      p.setTheta(-1 * p.getTheta()); 
      move(p); 
      // write to file < move-x move-y move-theta init-x init-y init-theta final-x final-y final-theta
      outfile << p.getX() << " " << p.getY() << " " << p.getTheta() << " "
	      << startPos.getX() << " " << startPos.getY() << " " << startPos.getTheta() << " "
	      << endPos.getX() << " " << endPos.getY() << " " << endPos.getTheta() << endl;
      sleep(1); 
    }
  }

  // close file
  outfile.close();
}






void Controller::move(Position p) {
      mot->move(p); 
      sleep(1);
      
      int count = 0;
      while ( !mot->isMoveCompleted(count) ){
      	usleep(200000);
			//cMan->Update();
      }
      usleep(100000);
      startPos = mot->getInitialPosition(); 
      endPos = mot->getFinalPosition(); 
}















void Controller::calibrateLinearMotion(BlackfinConfig *bc) 
{
	int targetDist[] = {5, 15, 25, 35, 45, 55, 65};
	bool done = false;
	int count = 0;
	int target, actual;
	double dactual;
	int tempi;
	double dx, dy;

	
	TBLUE("Starting Linear Calibrations\n\n\n");
	for(int i = 0; i < 7; i++) 
	{
		target = targetDist[i];
		Position p(target, 0, 0); 
		
      if(i == 0)tempi = bc->cm0_10;
      else if(i == 1)tempi = bc->cm11_20;
      else if(i == 2)tempi = bc->cm21_30;
      else if(i == 3)tempi = bc->cm31_40;
      else if(i == 4)tempi = bc->cm41_50;
      else if(i == 5)tempi = bc->cm51_60;
      else if(i == 6)tempi = bc->cm61_plus;
		
		while(done == false)
		{
      	move(p); 

			dx = startPos.getX() - endPos.getX();
			dy = startPos.getY() - endPos.getY();

			dactual = (sqrt( (dx * dx) + (dy * dy) ) + .5);
			actual = (int)dactual;
			
			int start = Utils::toDegrees(startPos.getTheta());
      	int end = Utils::toDegrees(endPos.getTheta());
      	if(start < 0) start = start + 360;
      	if(end < 0) end = end + 360;
			
			//cout << "target: " << target << endl;
			//cout << "actual: " << actual << endl << endl;
			
			int oldtempi = tempi;
			
			if(actual > (target + 10)) tempi = tempi + 5;
      	else if(actual > (target + 8)) tempi = tempi + 2;
      	else if(actual > (target + 6)) tempi = tempi + 1;
      	else if(actual > (target + 4)) tempi = tempi + 1;
      	else if(actual > (target + 2)) tempi = tempi + 1;
      	else if(actual < (target - 10)) tempi = tempi - 5;
      	else if(actual < (target - 8)) tempi = tempi - 2;
      	else if(actual < (target - 6)) tempi = tempi - 1;
      	else if(actual < (target - 4)) tempi = tempi - 1;
      	else if(actual < (target - 2)) tempi = tempi - 1;
      	else done = true;
      	
      	string temps;
      	      
      	if(i == 0){bc->cm0_10 = tempi; temps = "cm0_10";}
      	else if(i == 1){bc->cm11_20 = tempi; temps = "cm11_20";}
      	else if(i == 2){bc->cm21_30 = tempi; temps = "cm21_30";}
      	else if(i == 3){bc->cm31_40 = tempi; temps = "cm31_40";}
      	else if(i == 4){bc->cm41_50 = tempi; temps = "cm41_50";}
      	else if(i == 5){bc->cm51_60 = tempi; temps = "cm51_60";}
      	else if(i == 6){bc->cm61_plus = tempi; temps = "cm61_plus";}
      	
      	
      	TBLUE("----------------------------------\n");
      	cout << "target: " << target << endl;
			cout << "actual: " << actual << endl;
			cout << "Start: "<< startPos.getX() << ", " << startPos.getY() << ", " << start << endl;
			cout << "End: "<< endPos.getX() << ", " << endPos.getY() << ", " << end << endl;
			cout << temps << ": " << oldtempi << " -> " << tempi << "\n";
			//if(done == true) cout << temps << " Completed!\n";
			if(done == true) TGREEN(temps + " Completed\n");
			TBLUE("----------------------------------\n");
			cout << endl;
			
      	      	
      	bc->write();
      	Position turn(0,0,Utils::toRadians(180));
      	move(turn);


			bc->write();
      	usleep(200000);      	
      	
      	calibrating = true;
      		mot->reCalibrate();
      	calibrating = false;
      	
      	
      	count++;
      	if(count == 50) done = true;
		}
		done = false;
    	count = 0;
	}
	TGREEN("Calibrations Complete\n\n");
	return;
}







//186



void Controller::calibrateRotationalMotion(BlackfinConfig *bc) 
{
	setlocale(LC_ALL,"en_US.UTF-8");
  	int deltaTheta[] = {10, 30, 50, 70, 90, 110, 130, 150, 170};
	bool done = false;
	int count = 0;
	int target, actual;
   int start, end;
   double tempi;
   
   
   
   cout << "Waiting for Robot to be found...\n\n";
   
   Position tp(0,0,0);
  
   
   while(tp.getX() == 0)
   {
   	tp = mot->getPosition() ;
   	cout << tp.getX() << ", " << tp.getY() << ", " << tp.getTheta() << endl;
   	cMan->Update();
   	usleep(500000);
   }
    
   GREEN();
	cout << "\nRobot Found at: " << tp.getX() << ", " << tp.getY() << endl << endl;
	 cout << "test 1\n";
   WHITE();
   
   cout << "test 2\n";
   
	for(int i = 0; i < 9; i++) 
	{
      Position p(0,0,Utils::toRadians(deltaTheta[i]));
      target = deltaTheta[i];
      if(i == 0)tempi = bc->dg0_20;
      else if(i == 1)tempi = bc->dg21_40;
      else if(i == 2)tempi = bc->dg41_60;
      else if(i == 3)tempi = bc->dg61_80;
      else if(i == 4)tempi = bc->dg81_100;
      else if(i == 5)tempi = bc->dg101_120;
      else if(i == 6)tempi = bc->dg121_140;
      else if(i == 7)tempi = bc->dg141_160;
      else if(i == 8)tempi = bc->dg161_180;
      
      
      while(done == false)
      {
      	move(p); 
      	start = Utils::toDegrees(startPos.getTheta());
      	end = Utils::toDegrees(endPos.getTheta());
      	if(start < 0) start = start + 360;
      	if(end < 0) end = end + 360;
      	if(end < start) actual = ((360 - start) + end);
      	else actual = end - start;
      	

      	double oldtempi = tempi;
      	
      	if(actual > (target + 50)) tempi = tempi + 3;
      	else if(actual > (target + 20)) tempi = tempi + 2;
      	else if(actual > (target + 10)) tempi = tempi + 1.5;
      	else if(actual > (target + 8)) tempi = tempi + 1;
      	else if(actual > (target + 6)) tempi = tempi + .5;
      	else if(actual > (target + 4)) tempi = tempi + .25;
      	else if(actual > (target + 2)) tempi = tempi + .125;
      	else if(actual < (target - 50)) tempi = tempi - 3;
      	else if(actual < (target - 20)) tempi = tempi - 2;
      	else if(actual < (target - 10)) tempi = tempi - 1.5;
      	else if(actual < (target - 8)) tempi = tempi - 1;
      	else if(actual < (target - 6)) tempi = tempi - .5;
      	else if(actual < (target - 4)) tempi = tempi - .25;
      	else if(actual < (target - 2)) tempi = tempi - .125;
      	else done = true;
      	
      	string temps;
      	
      	
      	if(i == 0){bc->dg0_20 = tempi;temps = "dg0_20";}
      	else if(i == 1){bc->dg21_40 = tempi;temps = "dg21_40";}
      	else if(i == 2){bc->dg41_60 = tempi;temps = "dg41_60";}
      	else if(i == 3){bc->dg61_80 = tempi;temps = "dg61_80";}
      	else if(i == 4){bc->dg81_100 = tempi;temps = "dg81_100";}
      	else if(i == 5){bc->dg101_120 = tempi;temps = "dg101_120";}
      	else if(i == 6){bc->dg121_140 = tempi;temps = "dg121_140";}
      	else if(i == 7){bc->dg141_160 = tempi;temps = "dg141_160";}
      	else if(i == 8){bc->dg161_180 = tempi;temps = "dg161_180";}
      	
      	string d = "\u00B0";
      	//wchar_t d = 0x00B0;
      	
      	
      	//cout << "----------------------------------\n";
      	TBLUE("----------------------------------\n");
      	cout << "target: " << target << d << endl;
			cout << "actual: " << actual << d << endl;
			cout << "Start: "<< startPos.getX() << ", " << startPos.getY() << ", " << start <<  endl;
			cout << "End: "<< endPos.getX() << ", " << endPos.getY() << ", " << end << endl;
			cout << temps << ": " << oldtempi << " -> " << tempi << "\n";
			//if(done == true) cout << temps << " Completed!\n";
			if(done == true) TGREEN(temps + " Completed\n");
			TBLUE("----------------------------------\n");
			cout << endl;
      	//cout << "----------------------------------\n\n";
      	
      	
      	
      	      	
      	bc->write();
      	usleep(300000);
      	
      	calibrating = true;
      		mot->reCalibrate();
      	calibrating = false;
      	
      	count++;
      	if(count == 50) done = true;
      	
    }
    done = false;
    count = 0;
  }
  TGREEN("Rotation Calibrations Complete\n\n");
  return;
}



