#ifndef         CYBERGLOVE_CONTROL_H_
# define        CYBERGLOVE_CONTROL_H_

#define JOINTS_SIZE	20
#define DEFAULT_JOINT_ACTIVE	false

#include "cybergloveplus/cyberglove_calib_min_max.h"

namespace CyberGlovePlus
{

	class CyberGloveControl : public CyberGloveCalibMinMax
	{
		public:
			CyberGloveControl();
			~CyberGloveControl();

			int init();

			void run();

		protected:
			void action();

		private:
			std::vector<std::string> joints;
			std::map<std::string, double> j_values;
			std::map<std::string, double> j_previous_values;
			std::map<std::string, double> j_commands;
			std::map<std::string, double> j_previous_commands;
			std::map<std::string, int> min_angles;
			std::map<std::string, int> max_angles;
			std::map<std::string, bool> joints_active;

			std::map<std::string, ros::Publisher> publisher;

			static const double rad_ang = 3.141592 / 180;

			void init_limit_angles();
			void update_commands();
			void send_commands();

			bool send_command(std::string joint_name, double target);

			void linear_one_one(std::string joint, std::string sensor);
			double get_radiant_value(std::string joint, double value);
	};

}

#endif

