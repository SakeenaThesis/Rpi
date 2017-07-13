#include "ros/ros.h"
#include "pi2_ahs/PiToMaster.h"
#include <string>
#include <csignal>
#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pi2comms");
  ros::NodeHandle n;
  ros::Publisher pi2out_pub = n.advertise<pi2_ahs::PiToMaster>("pi2out", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    pi2_ahs::PiToMaster msg;

	if (count > 300)
	{
		msg.status = 1;
	}
	else
	{
		msg.status = 0;
	}
    
    msg.location = count;
    std::cout << "Status:"<< msg.status << "   " << "Location: " << msg.location << std::endl;
    pi2out_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

