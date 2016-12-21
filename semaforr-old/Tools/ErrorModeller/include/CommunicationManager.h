#ifndef _COMMUNICATION_MANAGER_H
#define _COMMUNICATION_MANAGER_H

#include "definitions.h"
#include "Motion.h"
#include "metrotimer.h"
#include "metrocommunication.h"
#include "libplayerc++/playerc++.h"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"
#include <cstring>
#include <stdint.h>
#include <stdlib.h>

using namespace metrobotics; 

class CommunicationManager
{
 public:
  CommunicationManager(PlayerCc::PlayerClient& pc, Motion * i, string label, string type);
  ~CommunicationManager();
  //CommunicationManager(const CommunicationManager& copy){}

  // State management.
  int  GetState() const { return mCurrentState; }
  bool IsRegistered() const { return mSessionID >= 0; }
  bool IsLocked() const { return mPossessed; }
  
  // Connect to the central server.
  bool Connect(const std::string& hostname, unsigned short port);
  void Disconnect();
  bool isCommAlive() const;

  void operator()(){
    while ( mCurrentState != STATE_QUIT ) {
      Update(); 
      usleep(10000);
    }
  } 
	
  // Heart beat of the robot unit;
  // Updates and maintains the internal state machine.
  void Update();
	
 private:
  // Binding to Player server
  PlayerCc::PlayerClient& mPlayerClient;

  // Player client proxies.
  boost::shared_ptr<PlayerCc::Position2dProxy> mPosition2D;

  // Interface to Motion
  Motion * mot;

  // CommunicationManager properties.
  std::string mNameID;
  std::string mTypeID;
  std::vector<std::string> mProvidesList;
  
  // Boost ASIO (for sockets)
  boost::asio::io_service mIOService;
  boost::asio::ip::tcp::socket mSocket;

  // State properties.
  int  mCurrentState;
  long mSessionID;
  bool mPossessed;

  // Internal timers.
  static const double MAX_TIME_SILENCE = 60.0;
  static const double MAX_TIME_STATE   = 10.0;
  static const double SWEEP_TIME = 0.0;
  metrobotics::PosixTimer mSilenceTimer;
  metrobotics::PosixTimer mStateTimer;

  // Internal buffers.
  std::string mStringBuffer;

  // Internal functions.
  void init_state();
  void init_provides();
  bool msg_waiting() const;
  bool read(std::stringstream& ss);
  bool write(const std::stringstream& ss);

  // State actions.
  void do_state_change(int state);
  void do_state_action_init();
  void do_state_action_ack();
  void do_state_action_idle();
  void do_state_action_ping_send();
  void do_state_action_pong_read();
  void do_state_action_pong_send();
  void do_state_action_cmd_proc();
  void do_state_action_pose();
  void do_state_action_player();
  void do_state_action_campose();
  void do_state_action_status();
};

#endif
