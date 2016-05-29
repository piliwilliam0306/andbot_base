
#define encoder0PinA  2
#define encoder0PinB  3

#define motorIn1 6
#define InA 5
#define InB 4
#define EN 7

#define LOOPTIME 40
#define FeedbackTime 100
#define CurrentLimit 8500 //for metal0

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

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
unsigned long lastSend = 0;                    // send timing 
long dT = 0;

double omega_target = 0.0;
double omega_actual = 0;

int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
double CPR = 28;                                   // encoder count per revolution
double gear_ratio = 65.5; 
int actual_send = 0;
int target_receive = 0;

float Kp = 0.9;
float Ki = 0.005;
float Kd = 0;
double error;
double pidTerm = 0;                                                            // PID correction
double sum_error, d_error=0;

double calculated_pidTerm;
double constrained_pidterm;

//current sense
int analogPin = A0;
unsigned int current = 0;
int current_send = 0;

//Motor Driver mode
int driver_mode = 0;
int driver_status = 0;

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
 //digitalWrite(EN, LOW);
 digitalWrite(EN, HIGH);
 Serial.begin (57600);
} 

void loop() 
{       
  readCmd_wheel_angularVel();
  //CurrentMonitor();
  if((millis()-lastMilli) >= LOOPTIME)   
     {                                    // enter tmed loop
        dT = millis()-lastMilli;
        lastMilli = millis();
        
        getMotorData();                                                           // calculate speed

        PWM_val = (updatePid(omega_target, omega_actual));                       // compute PWM value from rad/s 
        
        if (omega_target == 0)  { PWM_val = 0;  sum_error = 0; }
        
        if (PWM_val <= 0)   { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, LOW);  digitalWrite(InB, HIGH); }
        if (PWM_val > 0)    { analogWrite(motorIn1,abs(PWM_val));  digitalWrite(InA, HIGH);   digitalWrite(InB, LOW);}
     }
     
  if((millis()-lastSend) >= FeedbackTime)   
     {                                    // enter tmed loop
        lastSend = millis();
        sendFeedback_wheel_angularVel(); //send actually speed to mega
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
  //getMotorData();
  actual_send = int(omega_actual/0.00031434064); //convert rad/s to 16 bit integer to send
  //max current is 20400mA 20400/255 = 80
  current_send = current/80; 
  char sT='{'; //send start byte
  byte sH = highByte(actual_send); //send high byte
  byte sL = lowByte(actual_send);  //send low byte
  sP = '}'; //send stop byte 
  Serial.write(sT); Serial.write(sH); Serial.write(sL);  
  Serial.write(sP);
}

void getMotorData()  
{                               
  static long EncoderposPre = 0;       
  omega_actual = ((Encoderpos - EncoderposPre)*(1000/dT))*2*PI/(CPR*gear_ratio);  //ticks/s to rad/s
  EncoderposPre = Encoderpos;                 
}

void CurrentMonitor()
{
    // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
    current = analogRead(analogPin) * 34;  
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
   Serial.print("  dT:");                  Serial.print(dT);
   Serial.print("  sum_err:");                  Serial.print(sum_error);
   Serial.print("  Current:");                  Serial.print(current);
   Serial.print("  PWM_val:");                  Serial.print(PWM_val);

   Serial.println();
}

