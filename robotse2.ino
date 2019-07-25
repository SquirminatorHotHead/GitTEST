/***********************************************************
**project: wifi robot with webcam
************************************************************
*/
#include <SoftwareSerial.h>
#include <Servo.h>


 //* RX is digital pin 5 (connect to TX of other device - iRobot mini_din pin 4)
 //* TX is digital pin 6 (connect to RX of other device - iRobot mini_din pin 3)
int rxPin = 5;
int txPin = 6;

int LED1 = 8; 
int LED2 = 9; 

// set up a new software serial port:
SoftwareSerial softSerial =  SoftwareSerial(rxPin, txPin);

Servo myservo;  // create servo object to control a servo

int state; // determines if power is applied to sensor
int timed;
int pos = 45; // degree of servo motor

void setup()
{
  // irobot start part
  delay(2000); // NEEDED!!!! To let the robot initialize 
  
  // define pin modes for software tx, rx pins:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  // set the data rate for the SoftwareSerial port, this is the
  // iRobot's default rae of 57600 baud:
  softSerial.begin(57600);

  softSerial.write(128); // This command starts the OI. You must always send the Start command before sending any other commands to the OI
  softSerial.write(131); // safe mode 


  for(timed=0;timed<60;timed++)
  {
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,HIGH);
    delay(500);//delay, avoid noise signal while wifi start
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    delay(500);
  }

  // serial part
  Serial.begin(9600); //start serial communication
  state = 'e'; //default the sensor to off
  myservo.attach(11); // attaches the servo on pin 11
  myservo.write(45); // ini servo motor
  delay(1000);
  // Serial.println("move forward 1, back 2, left 3, right 4, stop 0");
}

void loop()
{
  // goForward();
  // delay(2000);
  // robotstop();
  // delay(1000);
  // goBack();
  // delay(2000);
  // robotstop();
  // delay(1000);
         
  //Serial.println("new loop ");
  if(Serial.available() > 0) //if data is being sent
  {
      Serial.println(state, DEC);
      state = Serial.read(); //read sent data
      Serial.println(state, DEC);
      
    if(state == 'e')
      robotstop();
    else if(state == 'a')
    {
      goForward();
      state = 0;
    }
    else if(state == 'b')
    {  
      goBack();
      state = 0;
    }
    else if(state == 'c')
    {  
      goRight();
      state = 0;
    }
    else if(state == 'd')
    {  
      goLeft();
      state = 0;
    }
    else if(state == 'j')
    {
      servoLeft();
    }
    else if(state == 'l')
    {
      servoRight();
    }
    else if(state == 'k')
    {
      servoStop();
    }
    else
      Serial.println("no command");
  }
}

void robotstop() {
  //Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
  softSerial.write(137);       // Opcode number for DRIVE
  // Velocity (-500 – 500 mm/s)
  softSerial.write((byte)0);   // 0x00c8 == 200  ff38 == -200
  softSerial.write((byte)0);
  // Radius (-2000 – 2000 mm)
  // Special case: straight = 32768 or 32767 = hex 8000 or 7FFF
  softSerial.write((byte)0);  // hex 80 
  softSerial.write((byte)0);    // hex 00
}

void goForward() {
  //Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
  softSerial.write(137);       // Opcode number for DRIVE
  // Velocity (-500 – 500 mm/s)
  softSerial.write((byte)0);   // 0x00c8 == 200  ff38 == -200
  softSerial.write((byte)200);
  // Radius (-2000 – 2000 mm)
  // Special case: straight = 32768 or 32767 = hex 8000 or 7FFF
  softSerial.write((byte)128);  // hex 80 
  softSerial.write((byte)0);    // hex 00
}

void goBack() {
  //Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
  softSerial.write(137);       // Opcode number for DRIVE
  // Velocity (-500 – 500 mm/s)
  softSerial.write((byte)255);   // 0x00c8 == 200  ff38 == -200
  softSerial.write((byte)56);
  // Radius (-2000 – 2000 mm)
  // Special case: straight = 32768 or 32767 = hex 8000 or 7FFF
  softSerial.write((byte)128);  // hex 80 
  softSerial.write((byte)0);    // hex 00
}

void goLeft() {
  //Serial sequence: [145] [right Velocity high byte] [right Velocity low byte] [left Velocity high byte] [left Velocity low byte]
  softSerial.write(145);       // Opcode number for DRIVE
  // right Velocity (-500 – 500 mm/s)
  softSerial.write((byte)0);   // 0x00c8 == 200  ff38 == -200
  softSerial.write((byte)150);
  // left Velocity (-500 – 500 mm/s)
  softSerial.write((byte)255);  // hex 80 
  softSerial.write((byte)106);    // hex 00
}

void goRight() {
  //Serial sequence: [145] [right Velocity high byte] [right Velocity low byte] [left Velocity high byte] [left Velocity low byte]
  softSerial.write(145);       // Opcode number for DRIVE
  // right Velocity (-500 – 500 mm/s)
  softSerial.write((byte)255);   // 0x00c8 == 200  ff38 == -200
  softSerial.write((byte)106);
  // left Velocity (-500 – 500 mm/s)
  softSerial.write((byte)0);  // hex 80 
  softSerial.write((byte)150);    // hex 00
}

void servoLeft() {
  if(pos>0){
    myservo.write(pos);
    pos -= 1;
    delay(15);
  }
}

void servoRight() {
  if(pos<90){
    myservo.write(pos);
    pos += 1;
    delay(15);
  }
}

void servoStop() {
}
