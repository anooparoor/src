#ifndef METROCOMMUNICATION_DEFINITIONS_H
#define METROCOMMUNICATION_DEFINITIONS_H

// Basic commands
#define CMD_INIT "INIT"
#define CMD_ACK  "ACK"
#define CMD_PING "PING"
#define CMD_PONG "PONG"


// Internal states.
#define STATE_INIT      0
#define STATE_ACK       1
#define STATE_PING_SEND 2
#define STATE_PONG_READ 3
#define STATE_PONG_SEND 4
#define STATE_QUIT      5
#define STATE_LISTEN    6


// Timeout values
#define MAX_TIME_SILENCE 60
#define MAX_TIME_STATE   10


#endif
