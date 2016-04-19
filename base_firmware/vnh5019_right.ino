//
#define encoder0PinA  2
#define encoder0PinB  3

#define motorIn1 6
#define InA 4
#define InB 5
#define EN 7

#define LOOPTIME 40

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

char commandArray[3];
byte sT = 0;  //send start byte
byte sH = 0;  //send high byte
byte sL = 0;  //send low byte
byte sCS = 0;  //send current sense byte
byte sP = 0;  //send stop byte

byte rT = 0;  //receive start byte
byte rH = 0;  //receive high byte
byte rL = 0;  //receive low byte
byte rP = 0;  //receive stop byte

volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;
unsigned long cc = 0;

double omega_target = 0.0;
double omega_actual = 0;

int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int CPR = 28;                                   // encoder count per revolution
int gear_ratio = 65.5; 
int actual_send = 0;
int target_receive = 0;

//float Kp = 1.0;
//float Ki = 0.03;
//float Ki = 0.005;

float Kp = 0.9;
//float Ki = 0.03;
float Ki = 0.005;

float Kd = 0;
double error;
double pidTerm = 0;                                                            // PID correction
double sum_error, d_error=0;

double calculated_pidTerm;
double constrained_pidterm;

//current sense
int analogPin = A0;
int current = 0;
int current_send = 0;

void setup() 
{ 
 //Set PWM frequency for D5 & D6
 // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
 TCCR0B = TCCR0B & B11111000 | B00000010;
 pinMode(encoder0PinA, INPUT); 
 digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
 pinMode(encoder0PinB, INPUT); 
 digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor

 attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
 attachInterrupt(1, doEncoder, CHANGE);
 pinMode(InA, OUTPUT); 
 pinMode(InB, OUTPUT); 
 pinMode(EN, OUTPUT);
 Serial.begin (57600);
} 

void loop() 
{       
  readCmd_wheel_angularVel();

  if((millis()-lastMilli) >= LOOPTIME)   
     {                                    // enter tmed loop
        dT = millis()-lastMilli;
        lastMilli = millis();
        
        getMotorData();                                                           // calculate speed

        sendFeedback_wheel_angularVel(); //send actually speed to mega
        
        PWM_val = (updatePid(omega_target, omega_actual));                       // compute PWM value from rad/s 

        //if (omega_target == 0) PWM_val = 0; 
	if (omega_target == 0)  { PWM_val = 0;  digitalWrite(EN, LOW);  }
        else                    digitalWrite(EN, HIGH);
        
        if (PWM_val <= 0)   { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, LOW);  digitalWrite(InB, HIGH); }
        if (PWM_val > 0)    { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, HIGH);   digitalWrite(InB, LOW);}
  
        //printMotorInfo();
     }
}

void readCmd_wheel_angularVel()
{
  if (Serial.available() >= 4) 
  {
    char rT = (char)Serial.read(); //read target speed from mega
          if(rT == '{')
            {
              char commandArray[3];
              Serial.readBytes(commandArray,3);
              byte rH=commandArray[0];
              byte rL=commandArray[1];
              char rP=commandArray[2];
              if(rP=='}')         
                {
                  target_receive = (rH<<8)+rL; 
                  omega_target = double (target_receive*0.00031434064);  //convert received 16 bit integer to actual speed
                }
            }
  }         
}

void sendFeedback_wheel_angularVel()
{
  actual_send = int(omega_actual/0.00031434064); //convert rad/s to 16 bit integer to send
  //current sense
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  current = analogRead(analogPin) * 34;
  current_send = current/136; //convert to 8 bit 1024*34/256=136
  char sT='{'; //send start byte
  byte sH = highByte(actual_send); //send high byte
  byte sL = lowByte(actual_send);  //send low byte
  char sP='}'; //send stop byte
  Serial.write(sT); Serial.write(sH); Serial.write(sL); 
  //Serial.write(sCS);//prepared for sending current drawing to mega 
  Serial.write(sP);
}

void getMotorData()  
{                               
  static long EncoderposPre = 0;       
  //converting ticks/s to rad/s
  omega_actual = ((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  EncoderposPre = Encoderpos;                 
}

double updatePid(double targetValue,double currentValue)   
{            
  
  static double last_error=0;                            
  error = targetValue - currentValue; 

  sum_error = sum_error + error * dT;
  // Added by KKuei to bound sum_error range
  sum_error = constrain(sum_error, -2000, 2000);
  
  d_error = (error - last_error) / dT;
  pidTerm = Kp * error + Ki * sum_error + Kd * d_error;   
  //pidTerm = Kp * error + Kd * d_error;                         
  last_error = error;  

  calculated_pidTerm = pidTerm/0.04039215686;
  constrained_pidterm = constrain(calculated_pidTerm, -255, 255);
  
  return constrained_pidterm;
}


void doEncoder() {
  pinAState = digitalRead(2);
  pinBState = digitalRead(3);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
      Encoderpos --;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
      Encoderpos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
      Encoderpos --;
  }
  pinAStateOld = pinAState;
  pinBStateOld = pinBState;
}

void printMotorInfo()  
{                                                                      
   Serial.print(" target:");                  Serial.print(omega_target);
   Serial.print(" actual:");                  Serial.print(omega_actual);
   //Serial.print(" sum_error:");              Serial.print(sum_error);
   //Serial.print("  error:");                  Serial.print(error);
   Serial.print("  samples:");                  Serial.print(cc);
   Serial.print("  dT:");                  Serial.print(dT);
   Serial.print("  sum_err:");                  Serial.print(sum_error);
   Serial.print("  Current:");                  Serial.print(current);
   Serial.print("  PWM_val:");                  Serial.print(PWM_val);

   Serial.println();
}

