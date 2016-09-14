# andbot_base

## MCU firmware
     mega_base.ino is for mega board.
     vnh5019_left.ino is for the motor controller board which has "L" label.  
     vnh5019_right.ino is for the motor controller board which has "R" label.
     Please select Arduino Pro Mini when uploading codes for the motor controller boards.
     Current sense in A0: 5V / 1024 ADC counts / 144 mV per A = 34 mA per count

## Modify Arduino PWM Frequency
     Add this line in setup()
          TCCR0B = TCCR0B & B11111000 | B00000010; // set timer 0 divisor to 8 for PWM frequency of 7812.50 Hz
     Overwrite wiring.c in: 
          ~/arduino-1.6.5/hardware/arduino/avr/cores/arduino
     Modify:
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
     To:
          #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))

## Note
     This wiring.c is only for motor control board, make sure to use original wiring.c when programming Mega 2560.
     The Arduino uses Timer 0 internally for the millis() and delay() functions, so be warned that changing the frequency of this timer will cause those functions to be erroneous.
     The frequency should, if possible, avoid the audio spectrum: below 20 Hz (not a good idea except for really massive motors) or above 20 KHz, so that the magnetostrictive vibration in windings or sympathetic vibration in the mechanical rotor, will not be heard by humans.

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

## Environment Setup (add in your notebook or desktop)

    $ echo "ROS_MASTER_URI=http://"MASTER_IP":11311" >>  ~/.bashrc

## View in Rviz (save banana.rviz in ~/catkin_ws/src)

    $ rosrun rviz rviz -d ~/catkin_ws/src/banana.rviz
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

http://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM

12345
