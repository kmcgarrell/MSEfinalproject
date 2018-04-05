#include <EEPROM.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>

#define DEBUG_ULTRASONIC
#define DEBUG1
#define DEBUG2

//required driving motor setup
Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_armswing;
Servo servo_knock;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
Servo servo_grab;

//port pin constants
const int Front_Ping = 10;   //input plug
const int Front_Data = 11;   //output plug
const int Side_Ping = 2; //input
const int Side_Data = 3; //output
const int Back_Ping = A1; //input
const int Back_Data = A0; //output
const int ci_Knock=4;
const int ci_PyramidPick = 12;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor=13;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5; 
const int bumperpin=5;
// I2C clock = yellow
int counter =0;
int NumTurns=0;
bool bumper=LOW;

//info for motor control
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//other varibles
unsigned long TimeFront;
unsigned long TimeSide;
unsigned long TimeBack;
int StateNum = 6;
boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;
boolean bt_Motors_Enabled = true;
long Left_Motor_Position;
long Right_Motor_Position;
const int ReferencePin=6;
int ReferenceState=LOW;
bool check=LOW;

int Side;
int Back;
int distance;
int state = 0;
const int target=17;
int error;
int output;
int constant=2;
int constant2=2;
int counter2=0;
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;


void setup()
{
  Wire.begin();
  Serial.begin(9600);
 

  // set up ultrasonics
  pinMode(Front_Ping, OUTPUT);
  pinMode(Front_Data, INPUT);
  pinMode(Side_Ping, OUTPUT);
  pinMode(Side_Data, INPUT);
  pinMode(Back_Ping, OUTPUT);
  pinMode(Back_Data, INPUT);
  pinMode(bumperpin, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_armswing.attach(ci_Arm_Motor);
  servo_knock.attach(ci_Knock);

  // set up motor enable switch
  pinMode(ci_PyramidPick, OUTPUT);
  servo_grab.attach(ci_PyramidPick);

  // set up motor encoders
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
}

void loop()
{

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_PyramidPick);
  servo_knock.write(180);
  servo_knock.detach();
  

  switch (StateNum)
  {
    case 1:
      {

        servo_armswing.writeMicroseconds(1500);
        servo_knock.write(180);
         // Ping();
          servo_LeftMotor.writeMicroseconds(1650);
          servo_RightMotor.writeMicroseconds(1650);
          Ping();
          Serial.print("Front Distance cm: ");
          Serial.println(TimeFront / 58);
          
          if((TimeFront/58)>20||(TimeFront/58)==0)
          {
            StateNum=1;
          }
          else
          {
            
          StateNum = 2;
          }
          break;
      }

    case 2:
      {
       
        Ping();

        int timemarker;
        int timemarker2;
        servo_armswing.writeMicroseconds(2500);
        servo_knock.write(180);
        if (counter==0)
        {
        timemarker2=millis();
        timemarker=millis();
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        }
        Left_Motor_Position = encoder_LeftMotor.getRawPosition();

        while(Left_Motor_Position<290)
        {
         servo_LeftMotor.writeMicroseconds(1650); 
         //servo_RightMotor.writeMicroseconds(1360); 
         servo_RightMotor.writeMicroseconds(1320); 
         Left_Motor_Position= encoder_LeftMotor.getRawPosition();
           
        }

        /*if(Left_Motor_Position<670)
        {
          Serial.print("Right Turn");
          Left_Motor_Position= encoder_LeftMotor.getRawPosition();
          servo_LeftMotor.writeMicroseconds(1680); 
          servo_RightMotor.writeMicroseconds(1360); 
          StateNum=2;
          counter++;
        }*/
          NumTurns++;
          if (NumTurns==8)
          {
            servo_LeftMotor.writeMicroseconds(1500);
            servo_RightMotor.writeMicroseconds(1500);
            
            StateNum=5;
            break;
          }
        
          StateNum=3;
          counter=0;
        

        /*servo_LeftMotor.writeMicroseconds(1680); 
        servo_RightMotor.writeMicroseconds(1360); 

        Ping();

        if((millis()-timemarker)<1000||(TimeSide/58)<=5)
        {
          StateNum=2;
          counter++;
        }

        else
        {
          StateNum=3;
          counter=0;
        }*/
        
        break;
      }


    case 3:
    {

     

     output=0;
     Ping();

     Back = (TimeBack/58);
     while(Back==0)
     {
      Ping();
      Back=(TimeBack/58);
      
     }

      Serial.print("Following Wall");
      servo_armswing.writeMicroseconds(2500);
      servo_knock.write(180);
      Side=(TimeSide/58);
      //Back=(TimeBack/58);
      distance=(TimeSide/58);

      if(Side!=target)
      {
        error=(Side-target);
        output=error*constant;
     
        servo_LeftMotor.writeMicroseconds(1610-output);
        servo_RightMotor.writeMicroseconds(1610+output);
        Ping();
      
      }

       if(Side>(target-0.5)&&Side<(target+0.5))
          {

              if(Side!=(Back+2))
            {
              error=Side-Back;
              output=error*constant2;
              
              servo_LeftMotor.writeMicroseconds(1610-output);
              servo_RightMotor.writeMicroseconds(1610+output);
              Ping();
            }
          }

      Ping();

      if((TimeFront/58)>20||(TimeFront/58)==0)
      {
        StateNum=3;
      }

      else
      {
        StateNum=2;
      }
      break;
      }
   case 5:
   {
      Serial.println("case 5"); 

  Ping();
  int state;
  int distance = (TimeFront/58);
  servo_knock.write(180);
  
  //1
  ReferenceState=digitalRead(ReferencePin);
  
  Serial.println(state);
  if (ReferenceState==HIGH)
  {
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    StateNum=6;
    Serial.println("found pyramid");
    Serial.println(StateNum);
servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);
Serial.println("going to case 6");
    break;
  }
  //2
  if (distance <20&&distance>0)
  {
    Serial.println("if 2");
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    Ping();
    distance = (TimeFront/58);
    Serial.println(distance);
    //3
    if (distance <20 && distance>0)
    {
      Serial.println("if 3");  
      int prev = millis();
      int current =millis();
      
      while ((current-prev)<1700)
      {
        servo_LeftMotor.writeMicroseconds(1600);
        servo_RightMotor.writeMicroseconds(1400);
        current=millis();
        ReferenceState=digitalRead(ReferencePin);
        if (ReferenceState==HIGH)
        {
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum=6;
          Serial.println("found pyramid");
          Serial.println(StateNum);
servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);

           break;
        }
        if (StateNum==6)
        break;
      }
      //delay(1500);

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      StateNum=5;
      break;
    }
      else 
  {
    Serial.println("we driving V2"); 
    if ((TimeSide/58)<20&&(TimeSide/58)>0)
    {
     int prev = millis();
      int current =millis();
      
      while ((current-prev)<1500)
      {
        servo_LeftMotor.writeMicroseconds(1600);
        servo_RightMotor.writeMicroseconds(1400);
        current=millis();
        ReferenceState=digitalRead(ReferencePin);
        if (ReferenceState==HIGH)
        {
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum=6;
          Serial.println("found pyramid");
          Serial.println(StateNum);
          servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);

           break;
        }
      }
      if (StateNum==6)
        break;
      //delay(1500);

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      StateNum=5;
      break;
    }
    
    servo_LeftMotor.writeMicroseconds(1620);
    servo_RightMotor.writeMicroseconds(1640);
    ReferenceState=digitalRead(ReferencePin);
    if (ReferenceState==HIGH)
        {
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum=6;
          Serial.println("found pyramid");
          Serial.println(StateNum);
          servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);

           break;
        }
    StateNum=5;
    break;
  }
    
  }
  else 
  {
    Serial.println("we driving");  
       if ((TimeSide/58)<20&&(TimeSide/58)>0)
    {
          int prev = millis();
      int current =millis();
      
      while ((current-prev)<1500)
      {
        servo_LeftMotor.writeMicroseconds(1600);
        servo_RightMotor.writeMicroseconds(1400);
        current=millis();
        ReferenceState=digitalRead(ReferencePin);
        if (ReferenceState==HIGH)
        {
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum=6;
          Serial.println("found pyramid");
          Serial.println(StateNum);
          servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);

           break;
        }
      }
      if (StateNum==6)
        break;
      //delay(1500);

      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1500);
      StateNum=5;
      break;
    }
    servo_LeftMotor.writeMicroseconds(1620);
    servo_RightMotor.writeMicroseconds(1640);

    StateNum=5;
     ReferenceState=digitalRead(ReferencePin);
    if (ReferenceState==HIGH)
        {
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum=6;
          Serial.println("found pyramid");
          Serial.println(StateNum);
        servo_grab.writeMicroseconds(1200);
delay(1000);
servo_grab.writeMicroseconds(1700);
delay(750);
servo_grab.writeMicroseconds(1500);
delay(5000);

           break;
        }
    StateNum=5;
    break;
  }
         
      //end of switch statements


    
   }

     
         
      //end of switch statements
 case 6:
      {
Serial.println("case 6");
ReferenceState=digitalRead(ReferencePin);
bumper=digitalRead(bumperpin);
delay(2000);
check = digitalRead(ReferencePin);
Serial.println(bumper);
if ( bumper == HIGH)
{
  Serial.println("bumper high eskettit");
servo_LeftMotor.writeMicroseconds(1500);
servo_RightMotor.writeMicroseconds(1500);
StateNum = 7;
delay(10000);
break;
}
else
{
   Serial.println("where the light @");
   if (ReferenceState==HIGH||check==HIGH)
{
  Serial.println("i see the light - driving towards");
servo_LeftMotor.writeMicroseconds(1600);
servo_RightMotor.writeMicroseconds(1620);
if ( bumper == HIGH)
{
  Serial.println("bumper high eskettit");
servo_LeftMotor.writeMicroseconds(1500);
servo_RightMotor.writeMicroseconds(1500);
StateNum = 7;
delay(10000);
break;
}
StateNum = 6;
break;

  
}
else
{
  Serial.println("trying to turn");
  do 
  {
   servo_LeftMotor.writeMicroseconds(1580);
   servo_RightMotor.writeMicroseconds(1420); 
   ReferenceState=digitalRead(ReferencePin); 
   if ( bumper == HIGH)
{
  Serial.println("bumper high eskettit");
servo_LeftMotor.writeMicroseconds(1500);
servo_RightMotor.writeMicroseconds(1500);
StateNum = 7;
delay(10000);
break;
}
  }
  while (ReferenceState!=HIGH);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);
  if (StateNum ==7)
  {
    break;
  }
  delay(1500);
  StateNum=6;
  break;  
  }
}
}




 case 7:
{
  //Serial.write("flip back");
  servo_grab.writeMicroseconds(1200);
  delay(1000); 

  //Serial.write("drive to optium position"); 
  servo_LeftMotor.writeMicroseconds(1650);
  servo_RightMotor.writeMicroseconds(1650);
  delay(1500);
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1500);


  
  //Serial.write("pick it up");
  servo_grab.writeMicroseconds(2200);
  delay(700);
  servo_grab.writeMicroseconds(1500);
  delay (1000);
  servo_grab.writeMicroseconds(1100);
  delay(700);
  servo_grab.writeMicroseconds(1500);
  delay(2000);

  //Serial.write("swing and hit cube");
 
  servo_armswing.writeMicroseconds(1500);
  delay(600);
  
  //Serial.write("knocking off the cube");
  servo_knock.write(180);
  delay(2000);
  servo_knock.write(120);
  delay(700);
  servo_knock.write(180);

  //Serial.write("swing back");
  servo_armswing.writeMicroseconds(2500);

  //Serial.write("hardcode locating the cube");
    //drive forward a bit
    //servo_LeftMotor.writeMicroseconds(1620);
    //servo_RightMotor.writeMicroseconds(1620);
    //delay(800);
    //turn a bit?
    servo_LeftMotor.writeMicroseconds(1430);
    servo_RightMotor.writeMicroseconds(1630);
    delay(1200);
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);

  //Serial.write("putting pyramid down");
  servo_grab.writeMicroseconds(1900);
  delay(800);
  servo_grab.writeMicroseconds(1500);
  delay (2000);
  //end of case 7
  
  StateNum=8;
  break;
}
}
}
         
      
    
    
      //end of switch statements
  

  


//-------------------- ULTRASONIC CODE ----------------------------//
void Ping()
{
  Serial.println("Pinging");
  digitalWrite(Front_Ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(Front_Ping, LOW);
  TimeFront = pulseIn(Front_Data, HIGH, 10000);

  digitalWrite(Side_Ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(Side_Ping, LOW);
  TimeSide = pulseIn(Side_Data, HIGH, 10000);

  digitalWrite(Back_Ping, HIGH);
  delayMicroseconds(10);
  digitalWrite(Back_Ping, LOW);
  TimeBack = pulseIn(Back_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Front Distance cm: ");
  Serial.println(TimeFront / 58); //divide time by 58 to get distance in cm 0
  Serial.print("Side Distance cm: ");
  Serial.println(TimeSide / 58); //divide time by 58 to get distance in cm 0
  Serial.print("Back Distance cm: ");
  Serial.println(TimeBack / 58); //divide time by 58 to get distance in cm 0
#endif
}
