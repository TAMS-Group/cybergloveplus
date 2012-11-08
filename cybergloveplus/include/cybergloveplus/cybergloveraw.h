# ifndef         CYBERGLOVERAW_H_
# define        CYBERGLOVERAW_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <pwd.h>
#include <sys/time.h>

#define GLOVE_SIZE 22
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_FREQUENCY   100


using namespace ros;

namespace CyberGlovePlus
{

	class CyberGloveRaw
	{
	public:
		CyberGloveRaw();
		~CyberGloveRaw();

		virtual int init();

		virtual void run();

	protected:
		ros::NodeHandle n_tilde;
		std::string gl_serial_port;
		int gl_frequency;
		int gl_button;

		std::map<std::string, double> gl_values;
		std::vector<std::string> gl_sensors;

		sensor_msgs::JointState jointstate_raw_msg;

		void read_sensor_values();
		void read_button_value();
		virtual void action();
		void print_params();

		std::string tolower(std::string);

	private:
		struct termios termios_save;
		int serial_port_fd;
		char glove_message[100];
		Publisher raw_pub;	

		int init_glove();
		int open_serial(std::string port);
		int read_stepping(int fd, unsigned char *b, int n);
		void  writeg(int fd, char *b, int n);

	};

}


# endif
