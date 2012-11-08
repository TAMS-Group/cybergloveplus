#include "../include/cybergloveplusui/formCybergloveCalibration.hpp"

namespace cybergloveplusui
{


FormCybergloveCalibration::FormCybergloveCalibration(QWidget *parent) : QDialog(parent), qnode()
{
    _ui.setupUi(this);

    _labels = _ui.frame->findChildren<QLabel*>();
    qDebug() << "found" << _labels.size() << "labels";

     _spinBoxes = _ui.frame->findChildren<QDoubleSpinBox*>();
    qDebug() << "found" << _spinBoxes.size() << "spinBoxes";

    _autoButtons = _ui.frame->findChildren<QCheckBox*>(QRegExp("_auto"));
    qDebug() << "found" << _autoButtons.size() << "auto checkBoxes";

    _selectButtons= _ui.frame->findChildren<QCheckBox*>(QRegExp("_exclusive"));
    qDebug() << "found" << _selectButtons.size() << "exclusive checkBoxes";

    qRegisterMetaType <GloveState> ("GloveState");
    connect(_ui.pushButton_send, SIGNAL(clicked()), this, SLOT(readGUI()));

	for (int i = 0; i < _selectButtons.size();i++)
	{
		_selectButtons[i]->setChecked(true);
		_autoButtons[i]->setChecked(true);
	}


	if (!qnode.init())
	{
		cout << "Could not init qnode" << endl;
	}

	 createInitialMap();

	timer = new QTimer(this);

	connect(timer, SIGNAL(timeout()), this, SLOT(refreshGUI()));
	timer->start(1000);

	ros_timer = new QTimer(this);

	connect(timer, SIGNAL(timeout()), this, SLOT(refresh_receive_ros()));

	ros_timer->start(10);
}


void FormCybergloveCalibration::createInitialMap()
{
    //QList<QLabel*> joints = _ui.frame->findChildren<QLabel*>();
    //qDebug() << "found" << joints.size() << "labels";
    for(unsigned int currentJoint=0; currentJoint < qnode.gl_sensors.size();currentJoint++)
	{
		QString sensor(qnode.gl_sensors[currentJoint].c_str());
		_sensorNames << sensor;
        //if(joints[currentJoint]->text().contains("G_")) _sensorNames << joints[currentJoint]->text();
    }
    qDebug() << _sensorNames;

    JointStatus status;
    status.autoCalibrate = true;
    status.selected = true;
    status.min = 0;
    status.max = 0;

    for(int currentSensorState=0; currentSensorState<_sensorNames.size();currentSensorState++)
	{
        _cyberGloveState[_sensorNames[currentSensorState]] = status;
    }

    qDebug() << "created a map of size" << _cyberGloveState.size();

    GloveState::iterator gloveItr;
    qDebug() << "********************************************************************";
    for(gloveItr=_cyberGloveState.begin(); gloveItr!=_cyberGloveState.end(); gloveItr++)
	{
        qDebug()<< "Sensor " << gloveItr->first << " has " << gloveItr->second.autoCalibrate;
    }
    qDebug() << "********************************************************************";

}

void FormCybergloveCalibration::print(GloveState state)
{
    GloveState::iterator gloveItr;
    qDebug() << "********************************************************************";
    for(gloveItr=state.begin(); gloveItr!=state.end(); gloveItr++){
        qDebug()<< "Sensor " << gloveItr->first << " has " << gloveItr->second.selected << " " << gloveItr->second.autoCalibrate << " " << gloveItr->second.min << " " << gloveItr->second.max;
    }
    qDebug() << "*******************************************************************";
}

JointStatus FormCybergloveCalibration::readJointStatusFromGUI(QString jointName)
{
    JointStatus currentStatus;
    currentStatus.autoCalibrate = false;
    currentStatus.selected = false;
    currentStatus.min = 0;
    currentStatus.max = 0;

    int currentRow = _sensorNames.indexOf(jointName);
    currentStatus.autoCalibrate = _autoButtons[currentRow]->isChecked();
    currentStatus.selected = _selectButtons[currentRow]->isChecked();
    currentStatus.min = _spinBoxes[currentRow*2]->value();
    currentStatus.max = _spinBoxes[currentRow*2+1]->value();

/*    qDebug() << "reading" << jointName //<< '\t'
                          << currentStatus.autoCalibrate << '\t'
                          << currentStatus.selected << '\t'
                          << currentStatus.min << '\t'
                          << currentStatus.max;
*/
    return currentStatus;
}


void FormCybergloveCalibration::readGUItoMap(){
    GloveState::iterator gloveItr;
    //qDebug() << "********************************************************************";
    for(gloveItr=_cyberGloveState.begin(); gloveItr!=_cyberGloveState.end(); gloveItr++){
        _cyberGloveState[gloveItr->first] = readJointStatusFromGUI(gloveItr->first);
    }
    //qDebug() << "********************************************************************";
}

GloveState FormCybergloveCalibration::getGloveState(){
    readGUItoMap();
    return _cyberGloveState;
}

void FormCybergloveCalibration::refresh_receive_ros()
{
	ros::spinOnce();
}


void FormCybergloveCalibration::refreshGUI()
{
	//print(_cyberGloveState);
	updateGUIfromGivenMap(qnode.current_status);

	for (unsigned int i = 0; i < qnode.gl_sensors.size(); i++)
	{
		QString text(qnode.gl_sensors[i].c_str());
		text += " ";
		text += QString::number(qnode.gl_values[i]);
		_labels[i]->setText(text);
	}
}


void FormCybergloveCalibration::setJointStatus(QString jointName)
{
    int currentRow = _sensorNames.indexOf(jointName);

    _autoButtons[currentRow]->setChecked(_cyberGloveState[jointName].autoCalibrate);
    _selectButtons[currentRow]->setChecked(_cyberGloveState[jointName].selected);
	if (_cyberGloveState[jointName].autoCalibrate)
	{
		_spinBoxes[currentRow*2]->setValue(_cyberGloveState[jointName].min);
		_spinBoxes[currentRow*2+1]->setValue(_cyberGloveState[jointName].max);
	}
}


void FormCybergloveCalibration::updateGUIfromGivenMap(GloveState newState){
    //_cyberGloveState = newState;
	readGUItoMap();
    GloveState::iterator gloveItr;

	//print(newState);
    for(gloveItr=newState.begin(); gloveItr!=newState.end(); gloveItr++)
    {
		if (! _cyberGloveState[gloveItr->first].selected)
			_cyberGloveState[gloveItr->first].autoCalibrate = newState[gloveItr->first].autoCalibrate;
		_cyberGloveState[gloveItr->first].min = newState[gloveItr->first].min;
		_cyberGloveState[gloveItr->first].max = newState[gloveItr->first].max;

        setJointStatus(gloveItr->first);
    }
}


void FormCybergloveCalibration::readGUI(){
	qnode.send_command(getGloveState());
}

}
