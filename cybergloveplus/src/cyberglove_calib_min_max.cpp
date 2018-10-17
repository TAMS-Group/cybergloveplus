#include "cybergloveplus/cyberglove_calib_min_max.h"

namespace CyberGlovePlus
{

CyberGloveCalibMinMax::CyberGloveCalibMinMax():CyberGloveRaw()
{
	run_once = true;
}

CyberGloveCalibMinMax::~CyberGloveCalibMinMax()
{
}

int CyberGloveCalibMinMax::init()
{
	int result = CyberGloveRaw::init();

	if (result != 0)
		return result;

	std::string prefix, full_topic, calibrated_full_topic;

	prefix = "/cybergloveplus";

	full_topic = prefix + "/calibrated/joint_states";
	value_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);

	full_topic = prefix + "/calibration/status";
	calib_pub = n_tilde.advertise<cybergloveplus::Calibration>(full_topic, 2);

	full_topic = prefix + "/calibration/command";
	calib_sub = n_tilde.subscribe(full_topic, 1000, &CyberGloveCalibMinMax::command_received_callback, this);

	full_topic = prefix + "/debug/";

	for (int i = 0; i < GLOVE_SIZE; i++)
	{
		std::string sensor = gl_sensors[i];
		min_values[sensor] = 0.4;
		max_values[sensor] = 0.6;
		auto_calibration[sensor] = DEFAULT_AUTO_CALIBRATION_ENABLED;

		std::string topic = full_topic + "raw_" + tolower(sensor);
		debug_raw_pub[sensor] = n_tilde.advertise<std_msgs::Float64>(topic, 2);

        topic = full_topic + "calib_" + tolower(sensor);
        debug_calib_pub[sensor] = n_tilde.advertise<std_msgs::Float64>(topic, 2);

        topic = full_topic + "min_" + tolower(sensor);
        debug_min_pub[sensor] = n_tilde.advertise<std_msgs::Float64>(topic, 2);

        topic = full_topic + "max_" + tolower(sensor);
        debug_max_pub[sensor] = n_tilde.advertise<std_msgs::Float64>(topic, 2);

	}

	min_values["G_WristYaw"] = 0.3;
	max_values["G_WristYaw"] = 0.35;

	return 0;
}

void CyberGloveCalibMinMax::command_received_callback(const cybergloveplus::Calibration::ConstPtr& msg)
{
	ROS_INFO("Message received");
	for (unsigned int i = 0; i < msg->name.size(); i++)
	{
		std::string sensor = msg->name[i];
		if (min_values.find(sensor) == min_values.end())
		{
			ROS_WARN_STREAM("Sensor "<< sensor << " not found.");
		}
		else
		{
			if (msg->auto_calibration[i])
				ROS_INFO("Command for %s auto enabled min=%f max=%f", sensor.c_str(), msg->min[i], msg->max[i]);
			else
				ROS_INFO("Command for %s auto disabled min=%f max=%f", sensor.c_str(), msg->min[i], msg->max[i]);
			auto_calibration[sensor] = msg->auto_calibration[i];
			min_values[sensor] = msg->min[i];
			max_values[sensor] = msg->max[i];
		}
	}
}

void CyberGloveCalibMinMax::action()
{
	CyberGloveRaw::action();

	if (run_once)
	{
		run_once = false;
		for (int i = 0; i < GLOVE_SIZE; i++)
		{
			std::string sensor = gl_sensors[i];
			min_values[sensor] = gl_values[sensor];
			max_values[sensor] = gl_values[sensor];
		}
	}

	calibrate();

	jointstate_calib_msg = jointstate_raw_msg;
	calib_msg.name = jointstate_calib_msg.name;

	jointstate_calib_msg.position.clear();
	jointstate_calib_msg.velocity.clear();

	calib_msg.min.clear();
	calib_msg.max.clear();
	calib_msg.auto_calibration.clear();

	for (int i = 0; i < GLOVE_SIZE; i++)
	{
		std::string sensor = gl_sensors[i];
		jointstate_calib_msg.position.push_back(gl_calibrated_values[sensor]);
		calib_msg.min.push_back(min_values[sensor]);
		calib_msg.max.push_back(max_values[sensor]);
		calib_msg.auto_calibration.push_back(auto_calibration[sensor]);

		std_msgs::Float64 vmsg;

		vmsg.data = gl_values[sensor];
		debug_raw_pub[sensor].publish(vmsg);

		vmsg.data = gl_calibrated_values[sensor];
		debug_calib_pub[sensor].publish(vmsg);
		
		vmsg.data = min_values[sensor];
		debug_min_pub[sensor].publish(vmsg);

		vmsg.data = max_values[sensor];
		debug_max_pub[sensor].publish(vmsg);
	}

	value_pub.publish(jointstate_calib_msg);
	calib_pub.publish(calib_msg);
}

void CyberGloveCalibMinMax::calibrate()
{
	for (int i = 0; i < GLOVE_SIZE; i++)
	{
		std::string sensor = gl_sensors[i];
		if (auto_calibration[sensor])
		{
			if (gl_values[sensor] < min_values[sensor]) min_values[sensor] = gl_values[sensor];
			if (gl_values[sensor] > max_values[sensor]) max_values[sensor] = gl_values[sensor];
		}

		gl_calibrated_values[sensor] = (gl_values[sensor] - min_values[sensor]) / (max_values[sensor] - min_values[sensor]);
	}	
}

}
