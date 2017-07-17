#include "ros/ros.h"
#include "pi2_ahs/PiToMaster.h"
#include "master_ahs/MasterToPis.h"

//for keyboard input
#include <csignal>
#include <termios.h>
#include <boost/thread.hpp>

	int pi2stat =0;
	int pi2loca =0;

// flags for instructions keyboard input
  bool quit_requested =false;
	bool pi1 = false;
	bool pi2 = false;
	bool pi3 = false;
	bool pi4 = false;
	bool pi5 = false;
	bool pi1manual = false;
	bool pi2manual = false;
	bool pi3manual = false;
	bool pi4manual = false;
	bool pi5manual = false;

	void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  int key_file_descriptor = 0;
	struct termios original_terminal_state;


// Pi2out callback
void pi2outCallback(const pi2_ahs::PiToMaster::ConstPtr& msg)
{
	pi2stat = msg->status;
	pi2loca = msg->location;
}

// keyboard input loop function thread
void keyboardInputLoop ()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("Select Raspberry Pi with numbers");
  puts("m : manual mode");
  puts("Space Bar : clear flags");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
   
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
    
  }

  puts("Exit");
}

void processKeyboardInput(char c)
{
  switch (c)
  {
    case '1':
    {
			pi1 = true;
			pi2 = false;
			pi3 = false;
			pi4 = false;
			pi5 = false;
      break;
    }
    case '2':
    {
			pi2 = true;
			pi1 = false;
			pi3 = false;
			pi4 = false;
			pi5 = false;
      break;
    }
    case '3':
    {
			pi3 = true;
			pi1 = false;
			pi2 = false;
			pi4 = false;
			pi5 = false;
      break;
    }
    case '4':
    {
			pi4 = true;
			pi1 = false;
			pi2 = false;
			pi3 = false;
			pi5 = false;
      break;
    }
    case '5':
    {
			pi5 = true;
			pi1 = false;
			pi2 = false;
			pi3 = false;
			pi4 = false;
      break;
    }
		case 32:
    {
			quit_requested =false;
			pi1 = false;
			pi2 = false;
			pi3 = false;
			pi4 = false;
			pi5 = false;
			pi1manual = false;
			pi2manual = false;
			pi3manual = false;
			pi4manual = false;
			pi5manual = false; 
      break;
    }
    case 'a':
    {
			if (pi1)
			{
				pi1manual = false;
			}
			else if(pi2)
			{
				pi2manual = false;
			}
			else if(pi3)
			{
				pi3manual = false;
			}
			else if(pi4)
			{
				pi4manual = false;
			}
			else if(pi5)
			{
				pi5manual = false;
			}
			else 
			{
				pi1manual = false;
				pi2manual = false;
				pi3manual = false;
				pi4manual = false;
				pi5manual = false;
			}
      break;
    }
    case 'm':
    {
			if (pi1)
			{
				pi1manual = true;
			}
			else if(pi2)
			{
				pi2manual = true;
			}
			else if(pi3)
			{
				pi3manual = true;
			}
			else if(pi4)
			{
				pi4manual = true;
			}
			else if(pi5)
			{
				pi5manual = true;
			}
			else 
			{
				pi1manual = false;
				pi2manual = true;
				pi3manual = false;
				pi4manual = false;
				pi5manual = false;
			}
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    default:
    {
      break;
    }
  }
}

int main(int argc, char **argv)
{
// initize node
  ros::init(argc, argv, "master_comms");

//make a node handle and subscribe to topic
  ros::NodeHandle n;

//pubs and subs
  ros::Publisher masterout_pub = n.advertise<master_ahs::MasterToPis>("masterout", 1000);
  ros::Subscriber sub = n.subscribe("pi2out", 1000, pi2outCallback);

// keyboard input thread
	tcgetattr(key_file_descriptor, &original_terminal_state);
  boost::thread t(boost::bind(&keyboardInputLoop));

// rate of looping
  ros::Rate loop_rate(10);

  int count  = 0; // this is just here while I dont have keyboard input

// loop of publishing and subscribing
  while (ros::ok())
  {
		// creating a master message
    master_ahs::MasterToPis msg;
		
		// altering the contents of the master message
    if (pi1manual)
		{
			msg.instruction_p1 = 1;
			msg.instruction_p2 = 0;
			msg.instruction_p3 = 0;
			msg.instruction_p4 = 0;
			msg.instruction_p5 = 0;
		}
    else if (pi2manual)
		{
			msg.instruction_p1 = 0;
			msg.instruction_p2 = 1;
			msg.instruction_p3 = 0;
			msg.instruction_p4 = 0;
			msg.instruction_p5 = 0;
		}
    else if (pi3manual)
		{
			msg.instruction_p1 = 0;
			msg.instruction_p2 = 0;
			msg.instruction_p3 = 1;
			msg.instruction_p4 = 0;
			msg.instruction_p5 = 0;
		}
    else if (pi4manual)
		{
			msg.instruction_p1 = 0;
			msg.instruction_p2 = 0;
			msg.instruction_p3 = 0;
			msg.instruction_p4 = 1;
			msg.instruction_p5 = 0;
		}
    else if (pi5manual)
		{
			msg.instruction_p1 = 0;
			msg.instruction_p2 = 0;
			msg.instruction_p3 = 0;
			msg.instruction_p4 = 0;
			msg.instruction_p5 = 1;
		}
		else
		{
			msg.instruction_p1 = 0;
			msg.instruction_p2 = 0;
			msg.instruction_p3 = 0;
			msg.instruction_p4 = 0;
			msg.instruction_p5 = 0;
		}
		msg.location_p2 = pi2loca;
    
		//publishing the master message
		masterout_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// printing stuff
		std::cout << "flags"<< std::endl;
		std::cout << "Pi1: "<< pi1 << "\t" << "manual: "<< pi1manual << std::endl;
		std::cout << "Pi2: "<< pi2 << "\t" << "manual: "<< pi2manual << std::endl;
		std::cout << "Pi3: "<< pi3 << "\t" << "manual: "<< pi3manual << std::endl;
		std::cout << "Pi4: "<< pi4 << "\t" << "manual: "<< pi4manual << std::endl;
		std::cout << "Pi5: "<< pi5 << "\t" << "manual: "<< pi5manual << std::endl;
		std::cout << "Status:"<< pi2stat << "\t" << "Location: " << pi2loca << std::endl;
    ++count; // this is just here while I dont have keyboard input
  }

  ros::spin();

  return 0;
}
