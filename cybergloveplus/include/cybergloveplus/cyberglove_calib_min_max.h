#ifndef         CYBERGLOVE_CALIB_MIN_MAX_H_
# define        CYBERGLOVE_CALIB_MIN_MAX_H_

#include "cybergloveplus/cybergloveraw.h"
#include <cybergloveplus/Calibration.h>

#define DEFAULT_AUTO_CALIBRATION_ENABLED	true

namespace CyberGlovePlus
{

	class CyberGloveCalibMinMax : public CyberGloveRaw
	{
		public:
			CyberGloveCalibMinMax();
			~CyberGloveCalibMinMax();

			virtual int init();

		protected:
			cybergloveplus::Calibration calib_msg;
			sensor_msgs::JointState jointstate_calib_msg;
			std::map<std::string, double> gl_calibrated_values;
			
			virtual void action();

		private:
			ros::Publisher calib_pub;
			ros::Subscriber calib_sub;
			std::map<std::string, ros::Publisher> debug_raw_pub;
			std::map<std::string, ros::Publisher> debug_calib_pub;
			std::map<std::string, ros::Publisher> debug_min_pub;
			std::map<std::string, ros::Publisher> debug_max_pub;
			ros::Publisher value_pub;
			std::map<std::string, double> max_values, min_values;
			std::map<std::string, bool> auto_calibration;
			bool run_once;
			
			void calibrate();
			void command_received_callback(const cybergloveplus::Calibration::ConstPtr& msg);
	};

} 


#endif
