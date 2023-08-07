#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Receiving Flag: %s", msg->data.c_str());


  // --- Do something based on the flag conditions --- //
  if (msg->data == "RED") // STOP CAR
  {
    ROS_INFO("STOP CAR");
  }
  
  if (msg->data == "YELLOW") // STOP CAR
  {
    ROS_INFO("WARNING");
  }

  if (msg->data == "GREEN") // STOP CAR
  {
    ROS_INFO("CONINUE");
  }
}


int main(int argc, char **argv)
{
  // Initialisng the subscriber node
  ros::init(argc, argv, "flag_status");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("flag_pub", 1000, chatterCallback);




  ros::spin();

  return 0;
}
 


