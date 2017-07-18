#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "pi2_ahs/PiToMaster.h"
#include "master_ahs/MasterToPis.h"
#include <string>
#include <csignal>
#include <sstream>


// variables
	int instruction = 0;
	int Acc =0;
	int Trate = 0;
	double AccFlo = 0.0;
	double TrateFlo = 0.0;

// joy callback function when pi recieves a joy menssage
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		//if in manual mode pass instructions
		if (instruction == 1)
			{
				AccFlo = 100*joy->axes[1];
				Acc = (int)AccFlo;
				TrateFlo = (-100)*joy->axes[3];
				Trate = (int)TrateFlo;
			}
		else
			{
				//do nothing
			}
	}

//Master topic subsctription callback
void masteroutCallback(const master_ahs::MasterToPis::ConstPtr& msg)
	{
		instruction = msg->instruction_p2;
	}


int main(int argc, char **argv)
{

		// defining a dummy variable for testing location updates
		int count = 0;

		ros::init(argc, argv, "pi2comms"); // initializing node
		std::cout << "Pi2"<< std::endl; // printing to show node has started running

		// node handles for subs and pubs
		ros::NodeHandle n; 
		ros::NodeHandle nh_;

		//Publisher for pi to master
		ros::Publisher pi2out_pub = n.advertise<pi2_ahs::PiToMaster>("pi2out", 1000);

		//Subscribing to joy messages and master messages
		ros::Subscriber joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
		ros::Subscriber sub = n.subscribe("masterout", 1000, masteroutCallback);

		// looping through listen for messages and taking action
		ros::Rate loop_rate(10);

	// looping through messages subs and pubs
		while (ros::ok())
			{
				pi2_ahs::PiToMaster msg;

				//***** Here we would update the status and location ****///
				// right now it's just updating based on count which increments each loop
				if (count > 300)
					{
						msg.status = 1;
					}
				else
					{
						msg.status = 0;
					}

				msg.location = count;

				// Print out things here
				std::cout << "Accel:"<< Acc << "   " << "TurnRate: " << Trate << std::endl;
				std::cout << "..."<< std::endl;
				std::cout << "Instruction:"<< instruction << std::endl;

				// Publish messages and spin callback functions
				pi2out_pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();

				//increment count
				++count;
			}

		return 0;
}

