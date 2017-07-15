#include "ros/ros.h"
#include "pi2_ahs/PiToMaster.h"

//topic subsctription callback
void pi2outCallback(const pi2_ahs::PiToMaster::ConstPtr& msg)
{
  std::cout << "Status:"<< msg->status << "   " << "Location: " << msg->location << std::endl;
}


int main(int argc, char **argv)
{
// initize node
  ros::init(argc, argv, "master_comms");

//make a node handle and subscribe to topic
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pi2out", 1000, pi2outCallback);

  ros::spin();


  return 0;
}


