#ifndef FORMCYBERGLOVECALIBRATION_H
#define FORMCYBERGLOVECALIBRATION_H

#include <QDialog>
#include <QDebug>
#include "cybergloveplusui/ui_cyberglove_calib.hpp"
#include "cybergloveplusui/qnode.hpp"
#include <QTimer>

using namespace std;

namespace cybergloveplusui
{

class FormCybergloveCalibration : public QDialog{
    Q_OBJECT
public:
    FormCybergloveCalibration(QWidget *parent = 0);
    void updateGUIfromGivenMap(GloveState newState);
   GloveState getGloveState();

private:

	QTimer* timer, *ros_timer;
	QNode qnode;
    Ui::Dialog_cybergloveCalib _ui;
    GloveState _cyberGloveState;
    QStringList _sensorNames;

    QList<QLabel*> _labels;
    QList<QDoubleSpinBox*> _spinBoxes;
    QList<QCheckBox*> _autoButtons;
    QList<QCheckBox*> _selectButtons;

    void createInitialMap();
    JointStatus readJointStatusFromGUI(QString jointName);
    void readGUItoMap();
	void setJointStatus(QString jointNam);

	void print(GloveState state);

//signals:
//    void SendDesiredGloveStateFromGUI(GloveState);

public slots:    
	void readGUI();
	void refreshGUI();
	void refresh_receive_ros();
};

}

#endif // FORMCYBERGLOVECALIBRATION_H
