#include <SoftwareSerial.h>

boolean result=false;
int SwitchState;
const int SwitchPin=4;
const int ReferencePin=5;

SoftwareSerial mySerial(7, 11); // RX, TX

void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("MSE 2202 IR tester");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
  //mySerial.println("Hello, world?");
  pinMode(SwitchPin, INPUT);
  //pinMode(ReferencePin, OUTPUT);
  //pinMode(ReferencePin, INPUT);
  pinMode(ReferencePin, OUTPUT);
}


void loop() {
  SwitchState=digitalRead(SwitchPin);
  Serial.write(SwitchState);
  if (SwitchState==HIGH)
  {
    //Serial.write("   searchingA-E-   ");
    char looking='A';
    char looking2='E';
    result=IRreading('A','E');
    char varible;
    if (result==true)
    {
      varible='h';
    }
    else
    {
      varible='n';
    }
    Serial.write(" Result is ");
    Serial.write(varible);
    Serial.println(result);
    if (result)
    {
      digitalWrite(ReferencePin, HIGH);
    }
    else 
    {
      digitalWrite(ReferencePin,LOW);
    }
  }
  if(SwitchState==LOW)
  {
    Serial.write("   searching-I-O-   ");
    char looking='I';
    char looking2='O';
    result=IRreading('I','O');
    Serial.println(result);
    if (result)
    {
      pinMode(ReferencePin, OUTPUT);
      digitalWrite(ReferencePin, HIGH);
    }
    else 
    {
      digitalWrite(ReferencePin,LOW);
    }
  }

}

//----------------------------IR CODE--------------------------------//

boolean IRreading(char looking, char looking2)
{
  char reading;
  if (mySerial.available())
  {
    Serial.println("IRreading");
    //Serial.write(mySerial.read());
    reading=mySerial.read();
    //Serial.write(reading);
    Serial.write(looking);
    //Serial.write("in the code");
    if (looking==reading || looking2==reading)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
    return false;
}





