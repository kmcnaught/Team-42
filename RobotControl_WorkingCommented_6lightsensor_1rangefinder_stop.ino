//////////////////////

// This code controls a light-folowing robot which uses an array of light sensor and proximity dectetors to follow light projected on the floor.
// Left and right sensors detect the direction of light, front and back make sure the robot is facing the right way, the range finder prevents the robot
// from bashing against the wall

// SET UP ////////////////////////////////////////////////////////////////////////
#define trigPin 2   // Trigger pin for ultrasound range finder
#define echoPin 3   // Echo pin for ultrasound range finder

// Variables
const int MotorPulse  = 500;  // Motor TTL pulse frequency

// Constants
const int SensorL     = A0;  // Left Light sensor pin
const int SensorR     = A1;  // Right Light sensor pin
const int SensorF     = A2;  // Front Light sensor pin
const int SensorT     = A3;  // Tail Light sensor pin


// Flags
float LeftValue          = 0;   // Flag for readout Left light Sensor
float RightValue         = 0;   // Flag for readout Right light Sensor
float FrontValue         = 0;   // Flag for readout Middle light Sensor
float TailValue          = 0;   // Flag for readout Middle light Sensor
float LeftSpeed          = 1;   // Flag Speed changes MotorPulse to modulate speed
float RightSpeed         = 1;   // Flag Speed changes MotorPulse to modulate speed
int LeftDel              = 0;   // Flag used to modulate left motor speed
int RightDel             = 0;   // Flag used to modulate right motor speed
int FrontDel             = 0;   // Flag used to modulate input from front light sensor
int TailDel              = 0;   // Flag used to modulate input from back light sensor
float TotLight           = 0.0; // Flag used to keep track of total light measured
float ServoMiddle        = 95.0;  // Flag used to modulate servo velocety depending of orientation
float Range              = 15.0;  // Flag used to measure range distance
int DirectionChoice      = 0;     // Flag used to choose direction when random choice is neessary

// Set up servo control
#include <Servo.h>
Servo MotorL;  // create servo object to control a servo
Servo MotorR;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position



// SETUP  //////////////////////////////////////////////////////////////////////////
void setup() {
  MotorL.attach(5);             // Define Left motor
  MotorR.attach(4);             // Define Right motor
  MotorL.write(ServoMiddle);    // Start Left motor
  MotorR.write(ServoMiddle);    // Start Left motor

  // Pin mode for range finder
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set up serial monitor
  Serial.begin(9600);
}


// LOOOOOP   /////////////////////////////////////////////////////////////////////////
void loop() {

  ///////// Check input from range finder ///////
  float duration, distance; // Send echo pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH); // Determine distance
  distance = (duration / 2) * 0.0344;


  ///////// Read light sensor values and adjust motor speed ///////
  LeftValue = analogRead(SensorL);
  RightValue = analogRead(SensorR);
  FrontValue = analogRead(SensorF);
  TailValue = analogRead(SensorT);
  TotLight = LeftValue + RightValue;

  // Make decision
  if ( (TailValue - FrontValue) / FrontValue > 0.2) {
    // You passed the light, go back
    MotorL.write(110);
    MotorR.write(110);
    delay(750);
  }
  else {
    if ( distance > 10 ) {
      // Not too close to a wall, follow the light!

      // Caluclate how to adjust motor speed to follow light
      LeftDel = (6 * (pow(RightValue / TotLight, 4) * Range) + ServoMiddle);
      RightDel = (ServoMiddle - (6 * pow(LeftValue / TotLight, 4) * Range));

      if (LeftDel > 110)
      {
        LeftDel = 110;
      }
      if (RightDel < 80)
      {
        RightDel = 80;
      }


      Serial.println(FrontValue);
      //Serial.println(RightDel));

      // if you're right under the light you can stop
      if (FrontValue > 10000) {
        MotorL.write(95);
        MotorR.write(95);
      }

      else {
        if ( abs(LeftDel - RightDel) < 12) {
          // Left Right light input difference too small, can't decide where to go
          // Go somewhere at random
          DirectionChoice = random(0, 2);
          Serial.println(DirectionChoice);

          if (DirectionChoice == 1) {
            MotorL.write(110);
            MotorR.write(110);
            delay(150);
            MotorL.write(110);
            MotorR.write(80);
            delay(200);
          }
          else {
            MotorL.write(80);
            MotorR.write(80);
            delay(150);
            MotorL.write(110);
            MotorR.write(80);
            delay(200);
          }
        }
        else {
          // We know where to go!
          MotorL.write(LeftDel);
          MotorR.write(RightDel);
          delay(100);
        }

      }
    }
    else {
      // If close to wall, back off and turn around
      MotorL.write(80);
      MotorR.write(110);
      delay(500);
      MotorL.write(110);
      MotorR.write(110);
      delay(500);
    }
  }


}
