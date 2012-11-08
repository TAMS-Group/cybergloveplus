/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/cybergloveplusui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cybergloveplusui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::QNode()
{
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
        gl_sensors.push_back("G_ThumbRotate");          //  0
        gl_sensors.push_back("G_ThumbMPJ");                     //  1
        gl_sensors.push_back("G_ThumbIJ");                      //  2
        gl_sensors.push_back("G_ThumbAb");                      //  3
        gl_sensors.push_back("G_IndexMPJ");                     //  4
        gl_sensors.push_back("G_IndexPIJ");                     //  5
        gl_sensors.push_back("G_IndexDIJ");                     //  6
        gl_sensors.push_back("G_MiddleMPJ");            //  7
        gl_sensors.push_back("G_MiddlePIJ");            //  8
        gl_sensors.push_back("G_MiddleDIJ");            //  9
        gl_sensors.push_back("G_MiddleIndexAb");        // 10
        gl_sensors.push_back("G_RingMPJ");                      // 11
        gl_sensors.push_back("G_RingPIJ");                      // 12
        gl_sensors.push_back("G_RingDIJ");                      // 13
        gl_sensors.push_back("G_RingMiddleAb");         // 14
        gl_sensors.push_back("G_PinkieMPJ");            // 15
        gl_sensors.push_back("G_PinkiePIJ");            // 16
        gl_sensors.push_back("G_PinkieDIJ");            // 17
        gl_sensors.push_back("G_PinkieRingAb");         // 18
        gl_sensors.push_back("G_PalmArch");                     // 19
        gl_sensors.push_back("G_WristPitch");           // 20
        gl_sensors.push_back("G_WristYaw");                     // 21

	gl_values.resize(22);

	ros::init(init_argc,init_argv,"cybergloveplusui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	std::string topic = "/cybergloveplus/calibration/command";
	// Add your ros communications here.
	chatter_publisher = n.advertise<cybergloveplus::Calibration>(topic, 1);

	topic = "/cybergloveplus/calibration/status";
	chatter_sub = n.subscribe(topic, 1, &QNode::status_received, this);

	topic = "/cybergloveplus/raw/joint_states";
	values_sub = n.subscribe(topic, 1, &QNode::values_received, this);

	start();
	return true;
}

void QNode::status_received(const cybergloveplus::Calibration::ConstPtr& msg)
{
	for (unsigned int i = 0; i < msg->name.size(); i++)
	{
		//std::string auto_c = msg->auto_calibration[i] ? "true" : "false";
		//std::cout << "Name = " << msg->name[i] << " " << auto_c << " " << msg->min[i] << " " << msg->max[i] << std::endl;
		QString sensor(msg->name[i].c_str());
		current_status[sensor].autoCalibrate = msg->auto_calibration[i];
		if (current_status[sensor].autoCalibrate)
		{
			current_status[sensor].min = msg->min[i];
			current_status[sensor].max = msg->max[i];
		}
                //std::string auto_c = current_status[sensor].autoCalibrate ? "true" : "false";
                //std::cout << "Name = " << sensor.toStdString() << " " << auto_c << " " << current_status[sensor].min << " " << current_status[sensor].max << std::endl;
	} 
}

void QNode::values_received(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (unsigned int i = 0; i < 22; i++)
	{
		gl_values[i] = msg->position[i];
	}
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"cybergloveplusui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {

		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "hello world " << count;
		//msg.data = ss.str();
		//chatter_publisher.publish(msg);
		//log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	emit loggingUpdated(); // used to readjust the scrollbar
}

void QNode::send_command(GloveState glovestate)
{
	cybergloveplus::Calibration msg;
	for (unsigned int i = 0; i < gl_sensors.size(); i++)
	{
		std::string sensor = gl_sensors[i];
		QString qsensor(sensor.c_str());
		if (glovestate[qsensor].selected)
		{
			msg.name.push_back(sensor);
			msg.auto_calibration.push_back(glovestate[qsensor].autoCalibrate);
			msg.min.push_back(glovestate[qsensor].min);
			msg.max.push_back(glovestate[qsensor].max);

			if (!glovestate[qsensor].autoCalibrate)
			{
				current_status[qsensor].min = glovestate[qsensor].min;
				current_status[qsensor].max = glovestate[qsensor].max;
			}
		}
	}
	chatter_publisher.publish(msg);
	ros::spinOnce();
}

}  // namespace cybergloveplusui
