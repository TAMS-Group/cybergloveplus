### cybergloveplus
ROS nodes for interfacing with our data glove / Vlad's cybergloveplus package ported to ROS

### Usage
- `rosparam set serialport /dev/ttyS0` : set correct port
- `rosrun cybergloveplus cyberglove_calib_min_max` : When the human master open and close his/her hand several times, this code will help to automatically calibrate the glove with the hand.
- `roslaunch cybergloveplus control.launch` : publish raw and calibrated joints values of the human hand, and remapping joints of shadow hand.
- `roslaunch cybergloveplus control_shadow_demo.launch` : teleoperate shadow hand in simulation
