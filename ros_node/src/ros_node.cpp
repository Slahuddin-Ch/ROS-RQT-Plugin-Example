#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>


uint32_t fib(uint32_t n);

uint32_t noToSend = 0;

void chatter1Callback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);

  ros::Subscriber sub = n.subscribe("chatter1", 1, chatter1Callback);

  ros::Rate loop_rate(0.5);


  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << fib(noToSend);
    msg.data = ss.str();



    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    


    ros::spinOnce();

    loop_rate.sleep();

    ++noToSend;
  }


  return 0;
}



uint32_t fib(uint32_t n) 
{ 
    if (n <= 1) 
        return n; 
    return fib(n-1) + fib(n-2); 
}

void chatter1Callback(const std_msgs::String::ConstPtr& msg)
{
 uint32_t recievedData = 0;
 
 recievedData = std::stoi(msg->data.c_str());

 if (recievedData == 1)
 {
   std::cout << "Reseting request Recieved\n";
   noToSend = 0; //start again

 }
  
}
