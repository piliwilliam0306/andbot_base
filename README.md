# andbot_base

## MCU firmware
  * mega_base.ino is for mega board.
  * vnh5019_left.ino is for the motor controller board which has "L" label.  
  * vnh5019_right.ino is for the motor controller board which has "R" label.

## ROS tutorials

    http://wiki.ros.org/ROS/Tutorials

## Copy Andbot packages to catkin workspace

    git clone https://github.com/piliwilliam0306/andbot_base.git
    cp src/ ~/catkin_ws/
  
## Build packages
    cd ~/catkin_ws
    catkin_make

## Launch Andbot nodes.

    roslaunch andbot andbot.launch

## Everything launched successfully
![](https://github.com/piliwilliam0306/andbot_base/blob/master/odom_received.png)

## View in Rviz
![](https://github.com/piliwilliam0306/andbot_base/blob/master/rviz.png)
