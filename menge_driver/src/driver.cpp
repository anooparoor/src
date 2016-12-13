#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "menge_commands");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);

     std::cout << "Type a command and then press enter. Use '+' to move forward, 'l' to turn left, 'r' to turn right, '.' to exit.\n";
     char cmd[50];

     //Declares the message to be sent
     geometry_msgs::Twist base_cmd;
     // Run the loop
     while(ros::ok()) {
	  std::cin.getline(cmd, 50);
          if(cmd[0]!='a' && cmd[0]!='w' && cmd[0]!='s' && cmd[0]!='d' && cmd[0]!='.'){
              std::cout << "unknown command:" << cmd << "\n";
              continue;
          }
          base_cmd.linear.x = base_cmd.linear.y = 0;
          //move left
          if(cmd[0]=='a'){
             base_cmd.linear.x = -0.15;
	     base_cmd.linear.y = 0.0;
           }
          //move right
          else if(cmd[0]=='d'){
             base_cmd.linear.x = 0.15;
             base_cmd.linear.y = 0.0;
          }
          //move up
          else if(cmd[0]=='w'){
             base_cmd.linear.x = 0.0;
             base_cmd.linear.y = -0.15;
          }
	  //move down
          else if(cmd[0]=='s'){
             base_cmd.linear.x = 0.0;
             base_cmd.linear.y = +0.15;
          }
          //quit
          else if(cmd[0]=='.'){
          break;
          }
 
          //publish the assembled command
          pub.publish(base_cmd);
          /*
          //Random x value between -2 and 2
          msg.linear.x=4*double(rand())/double(RAND_MAX)-2;
          //Random y value between -3 and 3
          msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
          //Publish the message
          pub.publish(msg);
	  */
          //Delays untill it is time to send another message
          rate.sleep();
         }
}
