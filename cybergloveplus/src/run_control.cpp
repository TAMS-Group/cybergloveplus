#include <ros/ros.h>
#include "cybergloveplus/cyberglove_control.h"

int main(int argc, char **argv)
{
        ros::init(argc, argv, "cybergloveplus_control");

        CyberGlovePlus::CyberGloveControl glove;

        int res;
        if ((res = glove.init()) == 0)
        {
                glove.run();
        }
        else
        {
                ROS_INFO("Could not init glove plus");
        }

    return 0;
}

