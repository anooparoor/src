#include <cv.h> /* required to use OpenCV */
#include <highgui.h> /* required to use OpenCV's highgui */
#include <stdio.h>
#include <termios.h>
#include <math.h>
#include <iostream>
#include <linux/joystick.h>
#include <pthread.h>
#include <fstream> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <errno.h>
//#include <assert.h>
//#include <sys/stat.h>



using namespace std; 

void Die(const char *mess) 
{ 
	perror(mess); exit(1); 
}

#include "Blackfin.h"

#include "JoyStick.h"

#include "Roomba.h"

#include "URG.h"

#include "URG_UG01.h"

#include "GPS.h"


