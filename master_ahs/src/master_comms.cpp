#include "ros/ros.h"
#include "pi2_ahs/PiToMaster.h"
#include "master_ahs/MasterToPis.h"

	int pi2stat =0;
	int pi2loca =0;

// Pi2out callback
void pi2outCallback(const pi2_ahs::PiToMaster::ConstPtr& msg)
{
	pi2stat = msg->status;
	pi2loca = msg->location;
}


int main(int argc, char **argv)
{
// initize node
  ros::init(argc, argv, "master_comms");

//make a node handle and subscribe to topic
  ros::NodeHandle n;

// publisher for master message to pis
  ros::Publisher masterout_pub = n.advertise<master_ahs::MasterToPis>("masterout", 1000);
// subscribes to messages from pi2
  ros::Subscriber sub = n.subscribe("pi2out", 1000, pi2outCallback);

// rate of looping
  ros::Rate loop_rate(10);

  int count  = 0; // this is just here while I dont have keyboard input

// loop of publishing and subscribing
  while (ros::ok())
  {
		// creating a master message
    master_ahs::MasterToPis msg;
		
		// altering the contents of the master message
    if (count > 300)
		{
			msg.instruction_p2 = 1;
		}
		else
		{
			msg.instruction_p2 = 0;
		}
    
		//publishing the master message
		masterout_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// printing stuff
		std::cout << "Status:"<< pi2stat << "\t" << "Location: " << pi2loca << std::endl;
    ++count; // this is just here while I dont have keyboard input
  }

  ros::spin();

  return 0;
}
