//This code uses direct port manipulation which only works on Arduino Mega 2560
//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <andbot/Bump.h>
#include <andbot/Sonar.h>
#include <avr/io.h>

const int bump1 = 23; //PA1
const int bump2 = 25; //PA3
const int bump3 = 27; //PA5
const int bump4 = 29; //PA7

const int cliff1 = 22; //PA0
const int cliff2 = 24; //PA2
const int cliff3 = 26; //PA4
const int cliff4 = 28; //PA6

const int TrigPin1 = 30;  //PC7
const int TrigPin2 = 31;  //PC6
const int TrigPin3 = 32;  //PC5
const int TrigPin4 = 33;  //PC4
const int TrigPin5 = 34;  //PC3
const int TrigPin6 = 35;  //PC2
const int TrigPin7 = 36;  //PC1
const int TrigPin8 = 37;  //PC0

const int EchoPin1 = 42;  //PL7
const int EchoPin2 = 43;  //PL6
const int EchoPin3 = 44;  //PL5
const int EchoPin4 = 45;  //PL4
const int EchoPin5 = 46;  //PL3
const int EchoPin6 = 47;  //PL2
const int EchoPin7 = 48;  //PL1
const int EchoPin8 = 49;  //PL0

char commandArray_L[3];
byte sT_L = 0;  //send start byte
byte sH_L = 0;  //send high byte
byte sL_L = 0;  //send low byte
byte sP_L = 0;  //send stop byte

byte rT_L = 0;  //receive start byte
byte rH_L = 0;  //receive high byte
byte rL_L = 0;  //receive low byte
byte rP_L = 0;  //receive stop byte

char commandArray_R[3];
byte sT_R = 0;  //send start byte
byte sH_R = 0;  //send high byte
byte sL_R = 0;  //send low byte
byte sP_R = 0;  //send stop byte

byte rT_R = 0;  //receive start byte
byte rH_R = 0;  //receive high byte
byte rL_R = 0;  //receive low byte
byte rP_R = 0;  //receive stop byte

#define LOOPTIME        100
#define BOOLTIME        1000 //1 Hz publish rate for cliff and bump sensor

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
unsigned long lastBool = 0;

long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

bool bump1_reading;
bool bump2_reading;
bool bump3_reading;
bool bump4_reading;
bool cliff1_reading;
bool cliff2_reading;
bool cliff3_reading;
bool cliff4_reading;
//Max.Distance(cm) = 70cm
unsigned long TimeOut = 5000;//TimeOut = Max.Distance(cm) * 58
unsigned long duration;
uint8_t cm;

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);

andbot::Bump bump_msg;
ros::Publisher pub_bump("bump", &bump_msg);

andbot::Sonar sonar_msg;
ros::Publisher pub_sonar( "sonar", &sonar_msg);

void messageCb(const geometry_msgs::Vector3& msg)
{
  omega_left_target = msg.x;  
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel",messageCb);

void setup() 
{

  //set baud rate for rosserial
  nh.getHardware()->setBaud(57600); 
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  nh.advertise(pub_sonar);
  nh.advertise(pub_bump);

  //Arduino Mega has three additional serial ports: 
    //Serial1 on pins 19 (RX) and 18 (TX), 
    //Serial2 on pins 17 (RX) and 16 (TX), 
    //Serial on pins 15 (RX) and 14 (TX). 
  //Serial.begin (57600);  
  Serial2.begin (57600);  //left
  Serial1.begin (57600);  //right
  DDRA &= ~0b11111111;  //set DDRA register as input for bump and cliff sensors
  DDRL &= ~0b11111111;  //set DDRL register as input for Echo pins
  DDRC |= 0b11111111;  //set DDRC register as output for Trig pins
}

void loop() 
{

  readFeadback_angularVel_L();
  readFeadback_angularVel_R();   
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          dT = millis()-lastMilli;
          lastMilli = millis();
          //send cmd to motor controller
          sendCmd_wheel_angularVel_L();
          sendCmd_wheel_angularVel_R();
          //publish actual speed
          vel_msg.x=omega_left_actual;
          vel_msg.y=omega_right_actual;
          p.publish(&vel_msg);

          //nh.spinOnce();
          //printMotorInfo();
       }     
  
    if((millis()-lastBool) >= BOOLTIME)   
       {
          lastBool = millis();

          //bump sensors
          bump1_reading = (PINA & (1<<PA1));
          bump_msg.bump1 = !bump1_reading;
          bump2_reading = (PINA & (1<<PA3));
          bump_msg.bump2 = !bump2_reading;
          bump3_reading = (PINA & (1<<PA5));
          bump_msg.bump3 = !bump3_reading;
          bump4_reading = (PINA & (1<<PA7));
          bump_msg.bump4 = !bump4_reading;
          
          //cliff sensors
          cliff1_reading = (PINA & (1<<PA0));
          bump_msg.cliff1 = !cliff1_reading;
          cliff2_reading = (PINA & (1<<PA2));
          bump_msg.cliff2 = !cliff2_reading;
          cliff3_reading = (PINA & (1<<PA4));
          bump_msg.cliff3 = !cliff3_reading;
          cliff4_reading = (PINA & (1<<PA6));
          bump_msg.cliff4 = !cliff4_reading;

          pub_bump.publish(&bump_msg);
          
          //sonar
          sonar_msg.sonar1 = ping(TrigPin1,EchoPin1);
          sonar_msg.sonar2 = ping(TrigPin2,EchoPin2);
          sonar_msg.sonar3 = ping(TrigPin3,EchoPin3);
          sonar_msg.sonar4 = ping(TrigPin4,EchoPin4);
          sonar_msg.sonar5 = ping(TrigPin5,EchoPin5);
          sonar_msg.sonar6 = ping(TrigPin6,EchoPin6);
          sonar_msg.sonar7 = ping(TrigPin7,EchoPin7);
          sonar_msg.sonar8 = ping(TrigPin8,EchoPin8);
          pub_sonar.publish(&sonar_msg);
        }  
  nh.spinOnce();
}

uint8_t ping(int TrigPin, int EchoPin) 
{
  digitalWrite( TrigPin, LOW );
  delayMicroseconds( 2 );
  digitalWrite( TrigPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( TrigPin, LOW );
  duration = pulseIn( EchoPin, HIGH,TimeOut);
  cm = (duration/2) / 29.1;
  return cm; 
}

void readFeadback_angularVel_L()
{
  if (Serial2.available() >= 4) 
  {
    char rT_L = (char)Serial2.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_L[3];
        Serial2.readBytes(commandArray_L,3);
        byte rH_L = commandArray_L[0];
        byte rL_L = commandArray_L[1];
        char rP_L = commandArray_L[2];
        if(rP_L == '}')         
          {
            left_actual_receive = (rH_L << 8) + rL_L; 
            omega_left_actual = double (left_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
          }
      }   
  }
}

void readFeadback_angularVel_R()
{
  if (Serial1.available() >= 4) 
  {  
    char rT_R = (char)Serial1.read(); //read actual speed from Uno
    if(rT_R == '{')
     {
       char commandArray_R[3];
       Serial1.readBytes(commandArray_R,3);
       byte rH_R = commandArray_R[0];
       byte rL_R = commandArray_R[1];
       char rP_R = commandArray_R[2];
       if(rP_R == '}')         
       {
        right_actual_receive = (rH_R << 8) + rL_R; 
        omega_right_actual = double (right_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
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
   Serial.print("  duration:");            Serial.print(duration);
   Serial.print("  cm:");                  Serial.print(cm);
   Serial.println(); 
   Serial.println(); 

}

*/


