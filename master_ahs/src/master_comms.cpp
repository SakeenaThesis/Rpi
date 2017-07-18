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
	bool manualPi1 = false;
	bool manualPi2 = false;
	bool manualPi3 = false;
	bool manualPi4 = false;
	bool manualPi5 = false;
	int selectPi = 0;
	int instrPi1 = 0;
	int instrPi2 = 0;
	int instrPi3 = 0;
	int instrPi4 = 0;
	int instrPi5 = 0;

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
			selectPi =1;
      break;
    }
    case '2':
    {
			selectPi =2;
      break;
    }
    case '3':
    {
			selectPi =3;
      break;
    }
    case '4':
    {
			selectPi =4;
      break;
    }
    case '5':
    {
			selectPi =5;
      break;
    }
		case 32: // reset space bar emergency stop
    {
			quit_requested =false;
			selectPi = 0;
			instrPi1 = 0;
			instrPi2 = 0;
			instrPi3 = 0;
			instrPi4 = 0;
			instrPi5 = 0;
      break;
    }
    case 'c': // clear selected pi
    {
			selectPi = 0;
      break;
    }
    case 'm':
    {
			if (selectPi == 1)
			{
				manualPi1 = !manualPi1;
				manualPi2 = false;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = false;
			}
			else if(selectPi == 2)
			{
				manualPi1 = false;
				manualPi2 = !manualPi2;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = false;
			}
			else if(selectPi == 3)
			{
				manualPi1 = false;
				manualPi2 = false;
				manualPi3 = !manualPi3;
				manualPi4 = false;
				manualPi5 = false;
			}
			else if(selectPi == 4)
			{
				manualPi1 = false;
				manualPi2 = false;
				manualPi3 = false;
				manualPi4 = !manualPi4;
				manualPi5 = false;
			}
			else if(selectPi == 5)
			{
				manualPi1 = false;
				manualPi2 = false;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = !manualPi5;
			}
			else 
			{
				manualPi1 = false;
				manualPi2 = !manualPi2;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = false;
			}
      break;
    }
		case 'f':
    {
			if (selectPi == 1)
			{
				instrPi1 = 2;
				manualPi1 = false;
			}
			else if(selectPi == 2)
			{
				instrPi2 = 2;
				manualPi2 = false;
			}
			else if(selectPi == 3)
			{
				instrPi3 = 2;
				manualPi3 = false;
			}
			else if(selectPi == 4)
			{
				instrPi4 = 2;
				manualPi4 = false;
			}
			else if(selectPi == 5)
			{
				instrPi5 = 2;
				manualPi5 = false;
			}
			else 
			{
				instrPi1 = 2;
				instrPi2 = 2;
				instrPi3 = 2;
				instrPi4 = 2;
				instrPi5 = 2;
				manualPi1 = false;
				manualPi2 = false;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = false;
			}
      break;
    }
		case 'd':
    {
			if (selectPi == 1)
			{
				instrPi1 = 3;
				manualPi1 = false;
			}
			else if(selectPi == 2)
			{
				instrPi2 = 3;
				manualPi2 = false;
			}
			else if(selectPi == 3)
			{
				instrPi3 = 3;
				manualPi3 = false;
			}
			else if(selectPi == 4)
			{
				instrPi4 = 3;
				manualPi4 = false;
			}
			else if(selectPi == 5)
			{
				instrPi5 = 3;
				manualPi5 = false;
			}
			else 
			{
				instrPi1 = 3;
				instrPi2 = 3;
				instrPi3 = 3;
				instrPi4 = 3;
				instrPi5 = 3;
				manualPi1 = false;
				manualPi2 = false;
				manualPi3 = false;
				manualPi4 = false;
				manualPi5 = false;
			}
      break;
    }
    case 'q':
    {
			instrPi1 = 0;
			instrPi2 = 0;
			instrPi3 = 0;
			instrPi4 = 0;
			instrPi5 = 0;
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
		if ((instrPi1 == 1) && (!manualPi1))
		{
			instrPi1 = 0;
		}
		if ((instrPi2 == 1) && (!manualPi2))
		{
			instrPi2 = 0;
		}
		if ((instrPi3 == 1) && (!manualPi3))
		{
			instrPi3 = 0;
		}
		if ((instrPi4 == 1) && (!manualPi4))
		{
			instrPi4 = 0;
		}
		if ((instrPi5 == 1) && (!manualPi5))
		{
			instrPi5 = 0;
		}
		// creating a master message
    master_ahs::MasterToPis msg;
		// altering the contents of the master message
    if (manualPi1)
		{
			instrPi1 = 1;
		}
    else if (manualPi2)
		{
			instrPi2 = 1;
		}
    else if (manualPi3)
		{
			instrPi3 = 1;
		}
    else if (manualPi4)
		{
			instrPi4 = 1;
		}
    else if (manualPi5)
		{
			instrPi5 = 1;
		}
		else
		{
			// change nothing
		}
		msg.instruction_p1 = instrPi1;
		msg.instruction_p2 = instrPi2;
		msg.instruction_p3 = instrPi3;
		msg.instruction_p4 = instrPi4;
		msg.instruction_p5 = instrPi5;

		msg.location_p2 = pi2loca;
    
		//publishing the master message
		masterout_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// printing here
		std::cout << "flags"<< std::endl;
		std::cout << "Pi selected: "<< selectPi << std::endl;
		std::cout << " 0 = stop \t 1 = manual \t 2 = pick up \t 3 = drop off "<< std::endl;
		std::cout << "Pi1 Instruction: "<< msg.instruction_p1 << std::endl;
		std::cout << "Pi2 Instruction: "<< msg.instruction_p2 << std::endl;
		std::cout << "Pi3 Instruction: "<< msg.instruction_p3 << std::endl;
		std::cout << "Pi4 Instruction: "<< msg.instruction_p4 << std::endl;
		std::cout << "Pi5 Instruction: "<< msg.instruction_p5 << std::endl;
		std::cout << "Status:"<< pi2stat << "\t" << "Location: " << pi2loca << std::endl;
    ++count; // this is just here while I dont have keyboard input
  }

  ros::spin();

  return 0;
}
