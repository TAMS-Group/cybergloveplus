
#include <ros/ros.h>

#include <string>
#include <sstream>

#include "cybergloveplus/cybergloveraw.h"

using namespace ros;

namespace CyberGlovePlus
{

CyberGloveRaw::CyberGloveRaw():n_tilde("~")
{
	gl_serial_port = DEFAULT_SERIAL_PORT;
	gl_frequency = DEFAULT_FREQUENCY;
}

CyberGloveRaw::~CyberGloveRaw()
{
}

int CyberGloveRaw::init()
{
	ROS_INFO("starting init");
	
	if (n_tilde.hasParam("serialport"))
	{
		n_tilde.getParam("serialport", gl_serial_port);
	}
	else if (n_tilde.hasParam("/serialport"))
	{
		n_tilde.getParam("/serialport", gl_serial_port);
	}
	ROS_INFO("Serial port is %s", gl_serial_port.c_str());

	if (n_tilde.hasParam("frequency"))
	{       
		n_tilde.getParam("frequency", gl_frequency);
	}
	else if (n_tilde.hasParam("/frequency"))
	{
		n_tilde.getParam("/frequency", gl_frequency);
	}
	ROS_INFO("Frequency is %d", gl_frequency);
	
	return init_glove();
}

void CyberGloveRaw::run()
{
	print_params();

	ros::Rate loop_rate(gl_frequency);

	while (ros::ok())
	{
		read_button_value();
		read_sensor_values();
		action();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void CyberGloveRaw::action()
{
	jointstate_raw_msg.position.clear();
	jointstate_raw_msg.velocity.clear();
	for(int i = 0; i < GLOVE_SIZE; i++)
	{
		jointstate_raw_msg.position.push_back(gl_values[gl_sensors[i]]);
	}
	raw_pub.publish(jointstate_raw_msg);
}

void CyberGloveRaw::print_params()
{
	ROS_INFO("Serial port is %s", gl_serial_port.c_str());
	ROS_INFO("Frequency is %d", gl_frequency);
}

int CyberGloveRaw::init_glove()
{
	serial_port_fd = -1;
		
	gl_sensors.push_back("G_ThumbRotate");		//  0
	gl_sensors.push_back("G_ThumbMPJ");			//  1
	gl_sensors.push_back("G_ThumbIJ");			//  2
	gl_sensors.push_back("G_ThumbAb");			//  3
	gl_sensors.push_back("G_IndexMPJ");			//  4
	gl_sensors.push_back("G_IndexPIJ");			//  5
	gl_sensors.push_back("G_IndexDIJ");			//  6
	gl_sensors.push_back("G_MiddleMPJ");		//  7
	gl_sensors.push_back("G_MiddlePIJ");		//  8
	gl_sensors.push_back("G_MiddleDIJ");		//  9
	gl_sensors.push_back("G_MiddleIndexAb");	// 10
	gl_sensors.push_back("G_RingMPJ");			// 11
	gl_sensors.push_back("G_RingPIJ");			// 12
	gl_sensors.push_back("G_RingDIJ");			// 13
	gl_sensors.push_back("G_RingMiddleAb");		// 14
	gl_sensors.push_back("G_PinkieMPJ");		// 15
	gl_sensors.push_back("G_PinkiePIJ");		// 16
	gl_sensors.push_back("G_PinkieDIJ");		// 17
	gl_sensors.push_back("G_PinkieRingAb");		// 18
	gl_sensors.push_back("G_PalmArch");			// 19
	gl_sensors.push_back("G_WristPitch");		// 20
	gl_sensors.push_back("G_WristYaw");			// 21

	for (int i = 0; i < GLOVE_SIZE; i++)
	{
		jointstate_raw_msg.name.push_back(gl_sensors[i]);
		gl_values[gl_sensors[i]] = 0;
	}

	std::string prefix, full_topic, calibrated_full_topic;

	prefix = "/cybergloveplus";

	full_topic = prefix + "/raw/joint_states";
	raw_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);

	return open_serial(gl_serial_port);
}	

int CyberGloveRaw::open_serial(std::string port) 
{
	struct termios termios_p;

	ROS_INFO("Opening %s\n",  port.c_str());
	serial_port_fd = open(port.c_str(), O_RDWR | O_NOCTTY); //will return a file descriptor based on an actual file
	
	if (serial_port_fd < 0) 
	{
		ROS_ERROR("Cannot open serial port! maybe run as root...");
		return 1;
	}

	ROS_INFO("Serial port opened succesfuly");
	
	//! get attributes associated with the serial port
	tcgetattr(serial_port_fd, &termios_p);
	
	//changes in termios_p won't affect termios_save behond this line...
	//memcpy(&termios_save, &termios_p, sizeof(struct termios)); //not used
	
	termios_p.c_cflag = B115200;
	termios_p.c_cflag |= CS8;
	termios_p.c_cflag |= CREAD;
	termios_p.c_iflag = IGNPAR | IGNBRK;
	termios_p.c_cflag |= CLOCAL;
	termios_p.c_oflag = 0;
	termios_p.c_lflag = 0; // NOT in canonical mode
	termios_p.c_cc[VTIME] = 1; //5; // half a second timeout
	termios_p.c_cc[VMIN] = 0;  // select timeout mode
	
	
	
	memcpy(&termios_save, &termios_p, sizeof(struct termios));
	  
	//! trying the following lines to restart serial port, see read_stepping() function
	tcsetattr(serial_port_fd, TCSANOW, &termios_p);
	tcflush(serial_port_fd, TCOFLUSH);  
	tcflush(serial_port_fd, TCIFLUSH);
	
	//  unsigned char out_data[2]={0xff, 0};
	//  write(serial_port_fd, out_data, 1);
	
	sleep(1);

	ROS_INFO ("Finished opening serial");
		
	return 0;
}

	/**
		called by read_values
	*	b is length GLOVE_SIZE + 2 because the first two characters returned are G and ' '
	*/
int CyberGloveRaw::read_stepping(int fd, unsigned char *b, int n) 
{
	int i=0;
	//fd_set fdset;
	int res=0;
	int remain;
	//int counter=0;
	remain = n;
	
	do 
	{
		res=read(fd, &b[i], remain);
		if (res>0) 
		{
			remain -= res;
			i += res;
		}
	
		if (res<0) 
		{
			ROS_ERROR("readg error!"); 
			return 1;
		}
		if (res==0) 
		{
			strcpy(glove_message, "(glove read failed, is it connected?)");
			//counter++; error(0,0,"returned no data");
			//HACK without this the glove will sometimes freeze
			tcsetattr(serial_port_fd, TCSANOW, &termios_save);
			return 1;
		}
	
		if (remain) 
		{
			usleep (remain * 500); //wait for more characters to appear on the port
		}
	} while ( /* (counter<10) && */ remain);
	
	return (remain!=0);
}
	
void  CyberGloveRaw::writeg(int fd, char *b, int n) 
{
	int i;
	for (i=0;i<n;i++) 
	{
		ssize_t result = write(fd, &b[i], 1);
		if (result != 1)
		{
			 ROS_WARN_STREAM("Serial write returned " << result);
		}
	}
	//  usleep(10*n);
}

//read the position values from the glove
void CyberGloveRaw::read_sensor_values() 
{
	while (1)
	{
		tcflush(serial_port_fd, TCIFLUSH);
		tcflush(serial_port_fd, TCOFLUSH);
		writeg(serial_port_fd, (char*)"G", 1);
		int i;
		unsigned char ch[GLOVE_SIZE+2]={0}; //assigns 0 to the first char
	
		usleep(GLOVE_SIZE*10);
		//  int res=read(serial_port_fd, &ch, GLOVE_SIZE+2);
		//  if (res<0) error(1,errno, "reading from serial port");
		if (read_stepping(serial_port_fd, (unsigned char*)&ch, GLOVE_SIZE+2)) 
		{
			//error(0,0,"serial port timeout");
			continue;
		}
		//  error(0,0,"res=%d, %02x %02x %02x %02x %02x %02x", res, ch[0],ch[1],ch[2],ch[3],ch[4],ch[5]);

		if (ch[0] != 'G') 
		{
			//error(0,0,"NO G");
			//log_message("%s: Failure reading glove: No G", timestamp());
			continue;
		}
	
		//! if any of the values are zero restart read as not considered reliable
		for (i=0; i<GLOVE_SIZE; i++) 
		{
  			if (ch[i+1]==0) 
			{
				//error(0,0,"Restart %d!", i);
				//log_message("%s: Restart %d", timestamp(), i);
				continue;
			}
		}

		for (i=0; i<GLOVE_SIZE; i++) 
		{
			gl_values[gl_sensors[i]] = (ch[i+1]-1.0)/254.0;

		}
		break;
	}
}

//read the button value from the glove
void CyberGloveRaw::read_button_value() 
{
	while (1)
	{
		tcflush(serial_port_fd, TCIFLUSH);
		tcflush(serial_port_fd, TCOFLUSH);
		writeg(serial_port_fd, (char*)"?W", 2);
		unsigned char ch[3]={0};    //assigns 0 to the first char

		//TODO : this should be reduced ? 
		usleep(GLOVE_SIZE*10);
		if (read_stepping(serial_port_fd, (unsigned char*)&ch, 3)) 
		{
			continue;
		}
			    
		gl_button = ch[2];
		break;
	}
}

std::string CyberGloveRaw::tolower(std::string inputstring)
{
        std::string result = inputstring;
        std::transform(result.begin(), result.end(), result.begin(), ::tolower);
        return result;
}


}
