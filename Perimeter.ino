#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>

#define DEBUG_ULTRASONIC

//required driving motor setup
Servo servo_RightMotor;
Servo servo_LeftMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//port pin constants
const int Front_Ping = 10;   //input plug
const int Front_Data = 11;   //output plug
const int Side_Ping = 2; //input
const int Side_Data = 3; //output
const int Back_Ping = 6; //input
const int Back_Data = 5; //output
const int ci_Motor_Enable_Switch = 12;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;

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
unsigned int StateNum = 0;
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
long l_Left_Motor_Position;
long l_Right_Motor_Position;
byte b_LowByte;
byte b_HighByte;
int counter=0;

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
  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2, ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonics
  pinMode(Front_Ping, OUTPUT);
  pinMode(Front_Data, INPUT);
  pinMode(Side_Ping, OUTPUT);
  pinMode(Side_Data, INPUT);
  pinMode(Back_Ping, OUTPUT);
  pinMode(Back_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up motor encoders
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      StateNum++;
      StateNum = StateNum & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      //ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);


  switch (StateNum)
  {
    case 0:
      {
        Ping();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        break;
      }
    case 1:
      {
          if (counter>0)
          {
            Ping();
            while (((TimeSide/58)>5) || (TimeBack/58 >7))
            {
               Ping();
               servo_LeftMotor.writeMicroseconds(1650);
               servo_RightMotor.writeMicroseconds(1500);
            }
          }
          Ping();
          while ((TimeFront/58) > 8)
          {
            servo_LeftMotor.writeMicroseconds(1600);
            servo_RightMotor.writeMicroseconds(1600);
            Ping();
          }
          servo_LeftMotor.writeMicroseconds(1500);
          servo_RightMotor.writeMicroseconds(1500);
          StateNum = 3;
          counter++;
          break;
      }

    case 2:
      {}

    case 3:
      {
        servo_LeftMotor.writeMicroseconds(1650);
        servo_RightMotor.writeMicroseconds(1500);
        delay(900);
        //int DistToWall=5;
        Ping();
        while (((TimeSide/58)>5) || (TimeBack/58 >7))
        {
          Ping();
          servo_LeftMotor.writeMicroseconds(1650);
          servo_RightMotor.writeMicroseconds(1500);
        }
        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
        StateNum = 1;
        break;
      }

      //end of switch statements
  }

  //end of loop
}


//-------------------- ULTRASONIC CODE ----------------------------//
void Ping()
{
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
  //Serial.print("Front Distance cm: ");
  //Serial.println(TimeFront / 58); //divide time by 58 to get distance in cm 0
  //Serial.print("Side Distance cm: ");
  //Serial.println(TimeSide / 58); //divide time by 58 to get distance in cm 0
  Serial.print("Back Distance cm: ");
  Serial.println(TimeBack / 58); //divide time by 58 to get distance in cm 0
#endif
}
