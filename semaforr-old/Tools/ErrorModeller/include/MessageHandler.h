#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include "CommunicationManager.h"

class MessageHandler {
 public:
  MessageHandler(CommunicationManager * c):cMan(c){}
  void operator()(){
    while ( cMan->GetState() != STATE_QUIT ){
      cMan->Update(); 
      usleep(10000);
    }
  }
 private:
    CommunicationManager * cMan; 
};

#endif
