/**
 * @file /include/cybergloveplusui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cybergloveplusui_QNODE_HPP_
#define cybergloveplusui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QString>
#include <QStringListModel>
#include <cybergloveplus/Calibration.h>
#include <sensor_msgs/JointState.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cybergloveplusui {

struct JointStatus{
    double max;
    double min;
    bool autoCalibrate;
    bool selected;
};

typedef std::map<QString, JointStatus> GloveState;


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	QNode();
	~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	void send_command(GloveState glovestate);
	
	GloveState current_status;
	std::vector<std::string> gl_sensors;
	std::vector<double> gl_values;


signals:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Subscriber chatter_sub, values_sub;
    QStringListModel logging_model;

	void status_received(const cybergloveplus::Calibration::ConstPtr& msg);
	void values_received(const sensor_msgs::JointState::ConstPtr& msg);
};

}  // namespace cybergloveplusui

#endif /* cybergloveplusui_QNODE_HPP_ */
