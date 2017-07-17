#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "pi2_ahs/PiToMaster.h"
#include "master_ahs/MasterToPis.h"
#include <string>
#include <csignal>
#include <sstream>


// variables
	int instruction;


// defining the joy teleop class
 class TeleopAHS
 {
	 public:
		 int instruction;
		 TeleopAHS();
	 
	 private:
		 void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	 
		 ros::NodeHandle nh_;
	 
		 int Acc, Trate;
		 double AccFlo, TrateFlo;
		 ros::Subscriber joy_sub_;
 };

// defining the joy teleop function
 TeleopAHS::TeleopAHS():
				Acc(0), Trate(0),
				AccFlo(0.0), TrateFlo(0.0)
 {
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAHS::joyCallback, this);
}

// joy callback function when pi recieves a joy menssage
 void TeleopAHS::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
 {
	/* when we start receiving instructions from master this will turn into a if statement. */
	if (instruction == 1)
	{
		AccFlo = 100*joy->axes[1];
		Acc = (int)AccFlo;
		TrateFlo = (-100)*joy->axes[3];
		Trate = (int)TrateFlo;
		std::cout << "Accel:"<< Acc << "   " << "TurnRate: " << Trate << std::endl;
		std::cout << "..."<< std::endl;
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
  std::cout << "Instruction:"<< instruction << std::endl;
}


int main(int argc, char **argv)
{

//defining a dummy variable for testing location updates
  int count = 0;
  instruction =0;

  ros::init(argc, argv, "pi2comms"); // initializing node
  std::cout << "Pi2"<< std::endl; // printing to show node has started running
  ros::NodeHandle n; //node handle for instriction

//teleoperation (might need if statement and destructor) 
  TeleopAHS teleop_AHS;
	teleop_AHS.instruction = 0;

// publishing pi status and location
  ros::Publisher pi2out_pub = n.advertise<pi2_ahs::PiToMaster>("pi2out", 1000);

// subscribing to master messages
  ros::Subscriber sub = n.subscribe("masterout", 1000, masteroutCallback);

// looping through listen for messages and taking action
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    teleop_AHS.instruction = instruction;
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
	//uncooment to print out location and status
    //std::cout << "Status:"<< msg.status << "   " << "Location: " << msg.location << std::endl;

	// publish messages and spin callback functions
    pi2out_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

	//increment count
    ++count;
  }


  return 0;
}

