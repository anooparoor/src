#ifndef _AUCTION_TASK_H
#define _AUCTION_TASK_H

class AuctionTask{
   double curr_x;
   double curr_y;
   double next_x;
   double next_y;
   string sensor;
   string task;
   int bid_id;
   double initial_cost;

 public:
   AuctionTask(){
      curr_x = curr_y = next_x = next_y = bid_id = -1;
      initial_cost = 0.0;
      sensor = task = "NONE";
   }
 
   AuctionTask(double cx, double cy, double nx, double ny, int i, string tsk, string snsr, double _cost){
      curr_x = cx;
      curr_y = cy;
      next_x = nx;
      next_y = ny;
      bid_id = i;
      task = tsk;
      sensor = snsr;
      initial_cost = _cost;
   }

   void setCurrLocation(double cx, double cy){
      curr_x = cx;
      curr_y = cy;
   }

   void setNextLocation(double nx, double ny){
      next_x = nx;
      next_y = ny;
   }

   void setID(int i){
      bid_id = i;
   }

   void setTask(string tsk){
      task = tsk;
   }

   void setSensor(string snsr){
      sensor = snsr;
   }

   void reset(){
      curr_x = curr_y = next_x = next_y = bid_id = -1;
      sensor = task = "NONE";
   }

   double getCurrX(){
      return curr_x;
   }

   double getCurrY(){
      return curr_y;
   }

   double getNextX(){
      return next_x;
   }

   double getNextY(){
      return next_y;
   }

   int getID(){
      return bid_id;
   }

   string getTask(){
      return task;
   }

   string getSensor(){
      return sensor;
   }

   double getInitialCost() {
      return initial_cost;
   }

   void setInitialCost(double _cost) {
      initial_cost = _cost;
   }

   void toString(){
      std::cout << " ******************************* " << std::endl;
      std::cout << " Bid id             : " << bid_id << std::endl;
      std::cout << " Current Location X : " << curr_x << std::endl;     
      std::cout << " Current Location Y : " << curr_y << std::endl;
      std::cout << " Next    Location X : " << next_x << std::endl;     
      std::cout << " Next    Location Y : " << next_y << std::endl;
      std::cout << " Task               : " << task << std::endl;
      std::cout << " Sensor             : " << sensor << std::endl;
      std::cout << " ******************************* \n" << std::endl;
   }
};

#endif
