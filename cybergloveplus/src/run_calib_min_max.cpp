#include "cybergloveplus/cyberglove_calib_min_max.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "cybergloveplus_calibminmax");

  CyberGlovePlus::CyberGloveCalibMinMax glove;

  int res;
  if ((res = glove.init()) == 0) {
    glove.run();
  } else {
    ROS_INFO("Could not init glove plus");
  }

  return 0;
}
