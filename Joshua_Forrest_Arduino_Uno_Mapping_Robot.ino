
/*
Joshua Forrest 3rd Year Mechatronnics Engineering Individual Project:
The Following code is intended for use with the accompanying python program. It is responsible for control of the robot, including command of the motors, all sensors and the messaging
*/

/* 
Inspiration for the timing-based approach and the servo sweep method can be found at http://forum.arduino.cc/index.php?topic=223286.0 , from Robin2's code for doing several things at a time.
Original Unmodified code for recvWithStartEndMarkers() and the processCommand() functions can be found at https://forum.arduino.cc/index.php?topic=288234.0 also originally made by Robin2.
The code for recvWithStartEndMarkers has been modified to allow splitting of an incoming string based upon detecting a specified character, in this case ';'. The maximum split is two.
*/

//Libraries
#include <Wire.h>
#include <Servo.h>
// Library from https://github.com/jrowberg/i2cdevlib
#include <I2Cdev.h>
// Library from https://github.com/pololu/vl6180x-arduino
#include <VL6180X.h>
// Library from https://github.com/Bill2462/MPU9255-Arduino-Library
#include <MPU9255.h>
//Initialisation of Parts
VL6180X Sensor;
Servo ScannerArm;
MPU9255 mpu;
//Variables and constants
  //Servo
unsigned long servoInterval = 50; //change for delay between servo movements. In milliseconds
int servoDegrees = 1; //servo steps
int servoMaxDegrees = 60;
int servoMinDegrees = 0;
int servoPosition = 30; //
unsigned long previousServoMillis = 0; // unsigned long to work with millis() function
unsigned long currentMillis = 0;
boolean sweep = true; //Controls wether the servo sweep or not
  //Mapping
float currentMovementMillis = 0;
unsigned long CommandInterval = 10;
unsigned long  previousCommandMillis = 0;
unsigned long rangeInterval = 50;
unsigned long previousRangeMillis = 0;
unsigned long mapInterval = 50;
unsigned long previousMappingMillis = 0;
unsigned long IMUInterval = 10;
unsigned long  previousIMUMillis = 0;
unsigned long previousCMDMillis = 0;
unsigned long CMDInterval = 2000;
float x = 0;
float startx = 0;
float y = 0;
float starty = 0;
float changer = 0;
int velocity = 200; //velocity in mm/s
int angularvelocity = 144; // in deg/s
const int turningRadius = 40; //distance from center of robot and wheels
const int z = 0.5; //distance between scanner and center of robot
float changeOrientation = 0;
float Orientation = 45;  // in degrees
float startOrientation = 0;
int alpha = 0; //servo arm current position 
uint8_t R;
uint8_t status;
  //Motor Functions
bool motion;  //Used to determine if robot is moving and therefore if IMU data needs to be sent
int in1 = 4;  //Pins 
int in2 = 5;
int in3 = 2;
int in4 = 7;
int ENA = 3;
int ENB = 6;
int MotorSpeed = 255; //PWM level
int Mot = 13; //Controls power to the Motor Driver, setting it LOW will turn it off
  //Mapping Data
unsigned long  previousMovementMillis = 0;
unsigned long  MovementInterval = 0;
unsigned long  actualMovementInterval = 0;
unsigned long startMovementMillis;
#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration
  // Variables used for incoming data
char message;
const byte maxDataLength = 20;          // maxDataLength is the maximum length allowed for received data.
char receivedChars[maxDataLength+1];
char receivedChars2[maxDataLength+1];
boolean newData = false;    
float pi2rad=(2*3.1415)/360;
//Function from the examples contained with the MPU9255 library
//process raw acceleration data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : acceleration in m/s^2
double process_acceleration(int input, scales sensor_scale )
{
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}
/*Function for parsing incoming data.*/
void recvWithStartEndMarkers() 
{
     static boolean recvInProgress = false;
     static boolean strSplit = false;
     static byte ndx = 0;
     static byte ndxs = 0;
     char startMarker = '<';
     char endMarker = '>';
     char splitMarker = ':';
     
     if (Serial.available() > 0 && newData == false) 
     {
          char rc = Serial.read();
          if (recvInProgress == true) //only true when first char was <
          {
               if (rc != endMarker) 
               {
                if (rc == splitMarker) {
                  strSplit = true;
                  }
                else {
                  if (strSplit == false) {if (ndx < maxDataLength) { receivedChars[ndx] = rc; ndx++;} }
                  else {if (ndxs < maxDataLength) { receivedChars2[ndxs] = rc; ndxs++;} }
                  }
               }
               else 
               {
                receivedChars[ndx] = '\0'; // terminate the string
                receivedChars2[ndxs] = '\0';
                recvInProgress = false;
                ndx = 0;
                ndxs = 0;
                newData = true;
                strSplit = false;
                }
          }
          else if (rc == startMarker) { recvInProgress = true; } //if < is first char set to true, loops to if statement
        
     }      
}
/*Function for updating of odometry based positioning. The function detects if the robot is in motion and monitiors the time and direction travelled to calculate updating positioning values*/
void locationupdate() 
{
  
  if (motion == true) {
    currentMovementMillis = currentMillis - startMovementMillis;
    if (strcmp("W",receivedChars) == 0) {changer = velocity*(currentMovementMillis/1000);}
    else if (strcmp("A",receivedChars) == 0) {changeOrientation = (angularvelocity*currentMovementMillis/1000);    
      }
    else if (strcmp("S",receivedChars) == 0) {changer = -(velocity*(currentMovementMillis/1000));}
    else if (strcmp("D",receivedChars) == 0) {changeOrientation = -(angularvelocity*currentMovementMillis/1000);
      }
    }
  else {if (changeOrientation != 0) {Orientation = startOrientation+changeOrientation;
    if (Orientation < 0) {Orientation = Orientation+360;}
    else if (Orientation >= 360 ) {Orientation = Orientation - 360;}}
    if (changer !=0) {
  x = startx+changer*cos(Orientation*(2*3.14/360));
  y = starty+changer*sin(Orientation*(2*3.14/360));}
  changer = 0;
  changeOrientation = 0;}
}

void setup() {
  Serial.begin(115200);
  
  ScannerArm.attach(9);
  ScannerArm.write(30);
  //The robot will wait until it recieves a start message to proceed with further setup
  while (strcmp ("START",receivedChars) != 0) {
  recvWithStartEndMarkers();
  if (newData)  {processCommand();} 
  }
  delay(50);
  Serial.println("Status;Starting");
  //Setup Servo
  ScannerArm.attach(9);
  ScannerArm.write(servoPosition);
  Serial.println("Status;Scanner Arm initialised");
  delay(50);   
  //Setup Motors
  pinMode(in1,OUTPUT); //Output to each motor
  pinMode(in2,OUTPUT);
  pinMode(ENA,OUTPUT); 
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(Mot,OUTPUT); //Used for controlling power to the motor driver
  digitalWrite(13,LOW);
  analogWrite(ENA, MotorSpeed); //Set PWM speeds
  analogWrite(ENB, MotorSpeed);
  digitalWrite(in1, LOW); //Start all motors off
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(Mot, HIGH);
  motion = false;
  Serial.println("Status;Motors initialised");
  delay(50);
  //Setup IMU
  mpu.init();
  Serial.println("Status;IMU intialiased");
  delay(50);
  //Setup Range Sensor
  Sensor.init();
  Sensor.configureDefault();
  Sensor.setTimeout(50);
  Serial.println("Status;Range Sensor initialiased");
  delay(50);
  Serial.println("Status;Ready to Begin Activity");
  delay(50);
  Serial.println("CMD;Ready for automated movement command");
  
}

void loop() {
  currentMillis = millis();
  StopCheck();
  CMDcheck();
  locationupdate();
  servoSweep();
  rangeread();
  accelread();
  recvWithStartEndMarkers(); 
  if (newData)  {processCommand();}   
}

//Function To check wether the robot has moved for long enough
void StopCheck() {
  if (((MovementInterval <= currentMillis - startMovementMillis) && motion == true)){ //or Sensor.readRangeSingle() <= 40) {
    actualMovementInterval = currentMillis - startMovementMillis;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    //if (Sensor.readRangeSingle() <= 40) {
    //  Serial.println("Status;Object detected");
    //  }
    if (actualMovementInterval >= MovementInterval) {
    Serial.print("TOF;");
    Serial.println(actualMovementInterval);
    }
    else {Serial.print("TOF;");
    Serial.println(MovementInterval);}
    motion = false;
  }
}

//Function to sweep the servo
void servoSweep() {
     
 if (currentMillis - previousServoMillis >= servoInterval && sweep == true) { // its time for another move
   previousServoMillis += servoInterval;
   servoPosition = servoPosition + servoDegrees; // servoDegrees might be negative
   
   if ((servoPosition == servoMaxDegrees) || (servoPosition == servoMinDegrees))  {
         // if the servo is at either extreme change the sign of the degrees to make it move the other way
     servoDegrees = - servoDegrees; // reverse direction
         // and update the position to ensure it is within range
     servoPosition = servoPosition + servoDegrees;
     }
 }
    // make the servo move to the next position
    ScannerArm.write(servoPosition);       
}

//Function to control the Range sensor and send current data about the robot
void rangeread() {
  
   if (currentMillis - previousRangeMillis >= rangeInterval ) {
    previousRangeMillis += rangeInterval;
    Serial.print("map;");
    Serial.print(Sensor.readRangeSingleMillimeters()); 
    Serial.print(",");
    if (motion == true) {
    Serial.print(startx+changer*cos(Orientation*(2*3.14/360)));
    }
    else {Serial.print(x);}
    Serial.print(",");
    if (motion == true) {
    Serial.print(starty+changer*sin(Orientation*(2*3.14/360)));
    }
    else {Serial.print(y);}
    Serial.print(",");
    Serial.print(servoPosition);
    Serial.print(",");
    if (motion == true) {
    Serial.println(startOrientation+changeOrientation);
    }
    else {Serial.println(Orientation);    
    }
    
    
  }
}
//Sends message to central control if ready for another command
void CMDcheck() {
  if (currentMillis - previousCMDMillis >= CMDInterval) {
    previousCMDMillis += CMDInterval;
    if (motion == false) {
      Serial.println("CMD;Ready");
      }
    }
  }
//Reads Accelerometer
void accelread() {
  if (currentMillis - previousMappingMillis >= mapInterval){
    previousMappingMillis += mapInterval;
    mpu.read_acc();
    Serial.print("Acc;");
    Serial.print(process_acceleration(mpu.ax,scale_2g));
    Serial.print(",");
    Serial.println(process_acceleration(mpu.ay,scale_2g));}
  }
//Processes Commands
void processCommand()
{
    newData = false;
    if (strcmp("STOP",receivedChars) == 0) {Serial.println(receivedChars2);}
    //Sets up data for odometry use
    if (strcmp("W",receivedChars) or strcmp("A",receivedChars) or strcmp("S",receivedChars) or strcmp("D",receivedChars) == 0) {
      startx = x;
      starty = y;
      startOrientation = Orientation;
      startMovementMillis = currentMillis;
      if (receivedChars2[0] != '\0') {
      MovementInterval = atoi(receivedChars2);
      }
    }
    // Command processes for motorcontrol, W, A, S, D and X (Forward, Backward, Rotate Counterclockwise, Rotate Clockwise)
    if      (strcmp ("W",receivedChars) == 0) {
      motion = true; 
      digitalWrite(Mot, LOW);     
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      digitalWrite(Mot, HIGH);  
      //Serial.println("Status;Forward"); 
      }
    else if (strcmp ("D",receivedChars) == 0) {       
      motion = true;
      digitalWrite(Mot, LOW);  
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      digitalWrite(Mot, HIGH);  
      //Serial.println("Status;Rotating Clockwise");
      }
    else if (strcmp ("S",receivedChars) == 0) {      
      motion = true;
      digitalWrite(Mot, LOW);  
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      digitalWrite(Mot, HIGH);  
      //Serial.println("Status;Backwards");
      }
    else if (strcmp ("A",receivedChars) == 0) {       
      motion = true;
      digitalWrite(Mot, LOW);  
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      digitalWrite(Mot, HIGH);  
      //Serial.println("Status;Rotating AntiClockwise");
      }
    else if (strcmp ("X",receivedChars) == 0) { 
      motion = false;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      //Serial.println("Status;Stop");
      }
    else if (strcmp ("Sweep",receivedChars) == 0) {
      sweep = !sweep;}

    //Commands to modify values such as speed, xpos etc.
    else if (strcmp ("SPD",receivedChars) ==0) {
      velocity = atoi(receivedChars2);
      Serial.println("Status;Speed Set");}    
    else if (strcmp ("ASPD",receivedChars) ==0) {
      angularvelocity = atoi(receivedChars2);
      Serial.println("Status;Rotation Speed Set");}
    else if (strcmp ("XPOS",receivedChars) ==0) {
      x = atof(receivedChars2);
      Serial.println("Status;X position Set");}
    else if (strcmp ("YPOS",receivedChars) ==0) {
      y = atof(receivedChars2);
      Serial.println("Status;Y position Set");}
    else if (strcmp ("ORI",receivedChars) ==0) {
      Orientation = atof(receivedChars2);
      Serial.println("Status;Orientation Set");}
}
