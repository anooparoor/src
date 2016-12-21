/* main.cpp: AuctionManager for metrobotics
 * Author: Ofear Balas <ofear.balas@gmail.com>
 *
 */

#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/thread.hpp>
//#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>

#include "defines.h"
#include "definitions.h"
#include "manager.h"
#include "robot_t.h"
#include "cqueue.h"

// Messaging
#include "CommunicationManager.h"
#include "MessageHandler.h"

//#define RANDOM_FLAG 1
#define ROBOT_NAME_STRING "blackfin"
#define OPTS "f:t:h:p:i:"

#define DEFAULT_CS_HOST "127.0.0.1"
#define DEFAULT_CS_PORT 6667

void sg_loop(cqueue<point_t >*);
void sg_file(cqueue<point_t>*, const char*);
void ran_loop(manager*, cqueue<point_t>*);
void man_loop(manager*, cqueue<point_t>*);
void robit_loop(int, int, int, cqueue<string >*);

int parse_ident(const string &ident_msg);
void parse_robots(char*, std::vector<string>&);
std::vector<string> parse_session_ids(const string &ident_msg);
void print_status(manager*);

int main(int argc, char* argv[]) {
  int opt;
  size_t auc_type = 0;
  char* filename = NULL;
  char* auc_ids = NULL;
  std::vector<string> robot_members;
  CommunicationManager *comm_mgr = NULL;
  AuctionManagerMessageHandler *message_handler = NULL;
  string my_name = "Auction_Manager";

  // TODO: read these from a configuration file
  string cs_host = DEFAULT_CS_HOST;
  uint32_t cs_port = DEFAULT_CS_PORT;

  while((opt = getopt(argc, argv, OPTS)) != -1) {
    switch(opt) {
    case 't':
      auc_type = atoi(optarg);
      fprintf(stderr, "Auction type chosen: %d\n", (int)auc_type);
      break;

    case 'f':
      filename = strndup(optarg, strlen(optarg)+1);
      filename[strlen(optarg)] = '\0';
      break;

    case 'h':
      cs_host = strndup(optarg, strlen(optarg)+1);
      cs_host[strlen(optarg)] = '\0';
      break;

    case 'p':
      cs_port = atoi(optarg);
      break;

    case 'i':
      auc_ids = strndup(optarg, strlen(optarg)+1);
      auc_ids[strlen(optarg)] = '\0';
      //TODO
      //parse_robots(auc_ids, robot_members);
      break;

    default:
      fprintf(stderr,"Usage %s -t auc_type\n", argv[0]);
      return 1;
    }
  }


  // TODO these messages need to be broadcast, not individual
  //cqueue<string> *robot_q = new cqueue<string>[32]; // Hai, two queues per robot (one up, one down)
  //
  cqueue<point_t > *goals_q = new cqueue<point_t>();
  /////////////////////////////////////SANDBOX

  // sg_loop pushes some random points/goals into the goals queue
  // if no points file is given

  boost::thread *sg_thread;
  if(filename != NULL)
    fprintf(stderr, "filename: %s\n", filename);
  if(filename == NULL)
    sg_thread = new boost::thread(&sg_loop, goals_q);
  ////////////////////////////////////////////
  else
    sg_thread = new boost::thread(&sg_file, goals_q, filename);

  //boost::thread *robit_threads[6];
  //

  fprintf(stderr, "Connecting to %s on %d...\n", cs_host.c_str(), cs_port);

  comm_mgr = new CommunicationManager(SID_COORDINATOR, my_name,
				      cs_host, cs_port);

  fprintf(stderr, "Starting Communication\n");
  comm_mgr->connect();

  boost::thread *comm_manager_thread = new boost::thread(boost::ref(*comm_mgr));

  sleep(1);

  message_handler = new AuctionManagerMessageHandler(comm_mgr);

  boost::thread *man_thread;
  manager *man = new manager(0, auc_type, message_handler);

  fprintf(stderr, "Manager started.\n");

  message_handler->set_manager(man);

  // Find out which robots are connected
  man->send_ident();

  // Idle until we get a response to the IDENT
  while ( man->get_robots() < 1 ) {
    man->update();
    usleep(1000);
  }

  if(auc_type == 0) { // Random
    man_thread = new boost::thread(&ran_loop, man, goals_q);
  }
  else {
    man_thread = new boost::thread(&man_loop, man, goals_q);
  }

  sg_thread->join();
  delete sg_thread;

  while ( man->num_auctions_completed() < 8 ) {
    sleep(1);
  }

  fprintf(stdout, "Run finished.\n");

  man_thread->interrupt();
  man_thread->join();
  delete man;

  comm_manager_thread->interrupt();
  comm_manager_thread->join();
  delete comm_manager_thread;

  return 0;
}

void sg_loop(cqueue<point_t> *bq) {
  boost::this_thread::disable_interruption di;
  fprintf(stderr, "Skygrid started.\n");
  srand(time(NULL));
  sstream stream;

  {
    for(int i = 0; i < (rand() % 2)+2 ; i++) {
      point_t p;
      p.first = rand() % 50*i+55;
      p.second = rand() % 50*i+55;
      bq->push(p);
      stream << "(" << p.first << ", " << p.second << ") ";
    }
  }
  fprintf(stderr, "Skygrid pushed points: %s\n", stream.str().c_str());

  while(true) {

    {
      if(!(rand() % 20)) {
	stream.str("");
	for(int i = 0; i < (rand() % 20 + 10); i++) {
	  point_t p;
	  p.first = rand() % 500;
	  p.second = rand() % 500;
	  bq->push(p);
	  stream << "(" << p.first << ", " << p.second << ") ";
	}
	fprintf(stderr, "Skygrid pushed points: %s\n", stream.str().c_str());
      }
    }

    sleep(4);
  }
}

void sg_file(cqueue<point_t> *bq, const char* filename) {
  bundle_t points;
  std::ifstream points_file(filename);
  if(!points_file.is_open()) {
    fprintf(stderr, "Points file not cannot be opened\n");
    return;
  }

  fprintf(stderr, "File opened, about to start parsing\n");
  while(!points_file.eof()) {
    double d;
    point_t p;
    points_file >> d;
    p.first = (int)d;
    if(points_file.eof()) // hack for badly made points files
      break;
    points_file >> d;
    p.second = (int)d;
    fprintf(stderr, "%d %d\n", p.first, p.second);
    points.push_back(p);
  }
  points_file.close();

  for(int i = 0; i < (int)points.size(); i++)
    bq->push(points[i]);
  return;
}

void ran_loop(manager *man, cqueue<point_t> *bq) {

  int next_robot = 0; // For round robin allocation

  bool resume_sent = false;

  std::vector<std::string> session_ids = man->get_robot_ids();

  // Pause all bots
  man->broadcast_wait();

  // For Random Allocation
  std::vector<std::pair<point_t, int> > allocated_points;
  bundle_t bund;
  while(true) {
    usleep(1000);
    // Check the message inbox
    man->update();

    if(!bq->empty()) {
      print_status(man);
      sstream points_stream;
      while(!bq->empty()) {
          point_t p = bq->front();
          bund.push_back(p);
          bq->pop();
          points_stream << "(" << p.first << ", " << p.second << ") ";

      }
      fprintf(stderr, "Manager got points: %s\n", points_stream.str().c_str());
    }

    for(size_t i = 0; i < bund.size(); next_robot++, i++) {
        if(next_robot >= man->get_robots())
            next_robot = 0;

        point_t p = bund[i];
        allocated_points.push_back(std::pair<point_t, int>(p, next_robot+1));
        fprintf(stderr, "Sending point to %s with index %d, p: (%d, %d)\n",
                session_ids[next_robot].c_str(), next_robot, p.first, p.second);
        man->send_cmd_auction_won(session_ids[next_robot], p.first, p.second);
    }

    bund.clear();

    // Let the bots start moving
    if (! resume_sent) {
      sleep(2);
      man->broadcast_resume();
      resume_sent = true;
    }

    // No more points? Get outta here
    if(boost::this_thread::interruption_requested())
        return;
  }
}

void man_loop(manager *man, cqueue<point_t> *bq) {

    bool resume_sent = false;

    bundle_t bund;

    // Pause all bots
    man->broadcast_wait();

    while(true) { // Main loop
      usleep(1000);
        man->update();

        if(!bq->empty()) {
            print_status(man);
            sstream points_stream;
            while(!bq->empty()) {
                point_t p = bq->front();
                bund.push_back(p);
                bq->pop();
                points_stream << "(" << p.first << ", " << p.second << ") ";

            }
            fprintf(stderr, "Manager got points: %s\n", points_stream.str().c_str());

            if(bund.size())
                man->do_auctioneer(bund);

            fprintf(stderr, "Manager bundled some points\n");
            bund.clear();
        }

        // Broadcast Pending Auctions
        for(int i = 0; i < man->auctions_size(); i++) {
            auction_t *auc = man->get_auction(i);
            if(auc->isresolved())
                continue;
            if(auc->iswaiting() && man->is_single()) // Do not broadcast more than one auction in single mode
                break;
            if(auc->isnew()) {
                sstream mess(auc->construct_message());

                auc->set_waiting();

                fprintf(stderr, "Broadcast Mess: %s\n",  mess.str().c_str()) ;

                // TODO: Make this cleaner
                //                commline->write(mess);
                man->send_message(mess.str());

                //(rq+i*2)->push(mess);

                // if we're in single auction mode then once we find a new
                // auction we need to break
                if(man->is_single())
                    break;
            }
        }

        // Let the bots start moving
        if (! resume_sent) {
            sleep(2);
            man->broadcast_resume();
	    resume_sent = true;
	}

        // If an interruption has been requested and there is no more work, exit
        if(boost::this_thread::interruption_requested() && man->ongoing_auctions() == 0) {
            return;
        }
    }

    //    delete msg_thread;
    //    delete msg_handler;
    //    delete commline;
}

int parse_ident(const string &ident_msg) {
    size_t pos = 0;

    if(ident_msg.size() <= 0)
        return 0;

    int blackfins = 0;
    while(true) {
        //fprintf(stderr, "pos: %u\n", pos);
        pos = ident_msg.find(ROBOT_NAME_STRING, pos);
        if(pos == string::npos)
            break;
        pos++;
        blackfins++;

    }

    return blackfins;
}

std::vector<string> parse_session_ids(const string &ident_msg) {
    std::vector<string> ids;
    size_t pos = 0;

    if(ident_msg.size() <= 0)
        return ids;

    while(true) {
        pos = ident_msg.find(ROBOT_NAME_STRING, pos);
        if(pos == string::npos)
            break;
        ids.push_back(
                      ident_msg.substr(ident_msg.rfind(' ', pos-2)+1, 10));
        pos++;
    }

    return ids;
}

void print_status(manager* man) {
    fprintf(stderr, "Status:\n");
    for(int i = 0; i < man->auctions_size(); i++) {
        auction_t *auc = man->get_auction(i);
        fprintf(stderr, "Auction: %5d Bundlesize: %5d Status: %15s\n",
                auc->get_id(), auc->bundle_size(),
                auc->get_status() == AUC_NEW  ? "AUC_NEW" :
                auc->get_status() == AUC_WAIT ? "AUC_WAIT" : "AUC_RESOLVED");
    }
}

