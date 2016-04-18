# andbot_base

## MCU firmware
     mega_base.ino is for mega board.
     vnh5019_left.ino is for the motor controller board which has "L" label.  
     vnh5019_right.ino is for the motor controller board which has "R" label.
     Please select Arduino Pro Mini when uploading codes for the motor controller boards.

## Modify Arduino PWM Frequency
     Add this line in setup()
     "TCCR1B = TCCR1B & B11111000 | B00000010;"    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
     Overwrite wiring.c in ~/arduino-1.6.5/hardware/arduino/avr/cores/arduino
     Modify
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
     To
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))

## Schematic
![](https://github.com/piliwilliam0306/andbot_base/blob/master/schematic/VNH5019.png)
![](https://github.com/piliwilliam0306/andbot_base/blob/master/schematic/ATMEGA328.png)

## ROS tutorials

    http://wiki.ros.org/ROS/Tutorials

## Copy Andbot packages to catkin workspace

    $ git clone https://github.com/piliwilliam0306/andbot_base.git
    $ cp -r src/ ~/catkin_ws/
  
## Build packages
    $ cd ~/catkin_ws
    $ catkin_make

## Launch Andbot nodes.

    $ roslaunch andbot andbot.launch

## Everything launched successfully
![](https://github.com/piliwilliam0306/andbot_base/blob/master/odom_received.png)

## View in Rviz
![](https://github.com/piliwilliam0306/andbot_base/blob/master/rviz.png)

## Simple Teleop operation
    $ sudo apt-get install ros-indigo-turtlebot-teleop

## modify turbotbot_teleop cmd_vel topic name
  * $ roscd turtlebot_teleop/launch
  * $ vi keyboard_teleop.launch
  * modify the following line
  * "remap from ="turtlebot_teleop_keyboard/cmd_vel" to="andbot/cmd_vel"/"
  * $ roslaunch turtlebot_teleop keyboard_teleop.launch
 
## Reference

https://arduino-info.wikispaces.com/Arduino-PWM-Frequency
http://playground.arduino.cc/Main/TimerPWMCheatsheet

