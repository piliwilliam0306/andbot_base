//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>

char commandArray_L[3];
byte sT_L = 0;  //send start byte
byte sH_L = 0;  //send high byte
byte sL_L = 0;  //send low byte
byte sP_L = 0;  //send stop byte

byte rT_L = 0;  //receive start byte
byte rH_L = 0;  //receive high byte
byte rL_L = 0;  //receive low byte
byte rCS_L = 0;  //receive current byte
byte rP_L = 0;  //receive stop byte

char commandArray_R[3];
byte sT_R = 0;  //send start byte
byte sH_R = 0;  //send high byte
byte sL_R = 0;  //send low byte
byte sP_R = 0;  //send stop byte

byte rT_R = 0;  //receive start byte
byte rH_R = 0;  //receive high byte
byte rL_R = 0;  //receive low byte
byte rCS_R = 0;  //receive current byte
byte rP_R = 0;  //receive stop byte

#define LOOPTIME        100

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

int Current_Draw_L = 0;
int Current_Draw_R = 0;

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);

void messageCb(const geometry_msgs::Vector3& msg)
{
  omega_left_target = msg.x;  
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel",messageCb);

void setup() 
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);

  //Arduino Mega has three additional serial ports: 
    //Serial1 on pins 19 (RX) and 18 (TX), 
    //Serial2 on pins 17 (RX) and 16 (TX), 
    //Serial on pins 15 (RX) and 14 (TX). 
  Serial2.begin (57600);  //left
  Serial1.begin (57600);  //right

}

void loop() 
{
  readFeadback_angularVel_L();
  readFeadback_angularVel_R();   
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          dT = millis()-lastMilli;
          lastMilli = millis();
 
          sendCmd_wheel_angularVel_L();
          sendCmd_wheel_angularVel_R();

          vel_msg.x=omega_left_actual;
          vel_msg.y=omega_right_actual;
          p.publish(&vel_msg);

          //nh.spinOnce();
          //printMotorInfo();
       }           
  nh.spinOnce();
}

void readFeadback_angularVel_L()
{
  //if (Serial2.available() >= 4) 
  if (Serial2.available() >= 5)   
  {
    char rT_L = (char)Serial2.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_L[4];
        Serial2.readBytes(commandArray_L,4);
        byte rH_L = commandArray_L[0];
        byte rL_L = commandArray_L[1];
	byte rCS_L = commandArray_L[2];
        char rP_L = commandArray_L[3];
        if(rP_L == '}')         
          {
            left_actual_receive = (rH_L << 8) + rL_L; 
            omega_left_actual = double (left_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
	    Current_Draw_L = rCS_L * 136;	
          }
      }   
  }
}

void readFeadback_angularVel_R()
{
  //if (Serial1.available() >= 4) 
  if (Serial1.available() >= 5)   
  {  
    char rT_R = (char)Serial1.read(); //read actual speed from Uno
    if(rT_R == '{')
     {
       char commandArray_R[4];
       Serial1.readBytes(commandArray_R,4);
       byte rH_R = commandArray_R[0];
       byte rL_R = commandArray_R[1];
       byte rCS_R = commandArray_L[2];
       char rP_R = commandArray_R[3];
       if(rP_R == '}')         
       {
        right_actual_receive = (rH_R << 8) + rL_R; 
        omega_right_actual = double (right_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
	Current_Draw_R = rCS_R * 136;	
       }  
     }
  }   
}

void sendCmd_wheel_angularVel_L()
{
  left_target_send = int(omega_left_target/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(left_target_send); //send high byte
  byte sL_L = lowByte(left_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_angularVel_R()
{
  right_target_send = int(omega_right_target/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT_R = '{'; //send start byte
  byte sH_R = highByte(right_target_send); //send high byte
  byte sL_R = lowByte(right_target_send);  //send low byte
  char sP_R = '}'; //send stop byte
  Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}

/*
void printMotorInfo()  
{                                                                      
   Serial.print("  LEFT target rad/s:");                  Serial.print(omega_left_target);
   Serial.print("  LEFT actual rad/s:");                  Serial.print(omega_left_actual);
   Serial.println(); 
   Serial.println(); 
   Serial.print("  RIGHT target rad/s:");                  Serial.print(omega_right_target);
   Serial.print("  RIGHT actual rad/s:");                  Serial.print(omega_right_actual);
   Serial.println(); 
   Serial.println(); 

}
*/



