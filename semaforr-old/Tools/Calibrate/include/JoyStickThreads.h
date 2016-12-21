

struct Joy
{
	const char* JOY_DEV;
	bool buttonsOn;
	double joyX;
	double joyY;
	double buttons;
	bool end;
};




void *JoyThread(void *SR)
{
	Joy *SS = (Joy *)SR;
	
	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
	char *button=NULL, name_of_joystick[80];
	struct js_event js;
	double joyButtons = 0;
	unsigned char c;
	int i = 0;
	int but[12];
	
	if( ( joy_fd = open( SS->JOY_DEV , O_RDONLY)) == -1 )
	{
		printf( "Couldn't open joystick\n" );
		printf( "Make sure joystick is plugged into usb and %s is availabe\n", SS->JOY_DEV );
		SS->buttons = 244;
		pthread_exit(NULL);
	}
	
	printf( "Joy Stick connected on %s\n", SS->JOY_DEV );

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */
	
		
	
	while(SS->end == false)
	{
		read(joy_fd, &js, sizeof(struct js_event));
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				break;
			case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				break;
		}
		
		SS->joyX = axis[0] * -1;
		SS->joyY = axis[1] * -1;
		
		for( x=0 ; x<num_of_buttons ; ++x )
		{
			c = button[x];
			but[x] = c;			
		}

		SS->buttons = but[0] + but[1]*2 + but[2]*pow(2,2) + but[3]*pow(2,3) + but[4]*pow(2,4) + 
			but[5]*pow(2,5) + but[6]*pow(2,6) + but[7]*pow(2,7) + but[8]*pow(2,8) + but[9]*pow(2,9) + 
			but[10]*pow(2,10) + but[11]*pow(2,11);	

	}
	printf("Closing connection to JoyStick\n");
	close( joy_fd );
	pthread_exit(NULL);

}

