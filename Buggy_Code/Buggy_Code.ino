#include "config.h"
#include "motors.h"
#include "ultrasonic.h"
#include "wheelencoders.h"
#include "PIDcontroller.h"
#include "secrets.h"
#include <WiFiS3.h>
#include <PID_v1.h>

#define PI 3.14159


// Define IR sensor threshold
const int THRESHOLD = 500;
extern bool justChanged;
extern bool going, stopped;

// Define ultrasonic sensor parameters
const unsigned long ULTRASONIC_INTERVAL = 100;  // Interval for ultrasonic readings (ms)
const int STOP_DISTANCE = 20;                   // Distance in cm at which the buggy should stop

// Ultrasonic Sensor
UltrasonicSensor ultrasonic(ULTRA_SONIC_TRIG, ULTRA_SONIC_ECHO);

unsigned long lastUltrasonicTime = 0;
bool objectDetected = false;

PIDController followPID(0.02, 0.5, 0.2); // Distance following PID
PIDController speedPID(0.5, 0.02, 0.2); // Speed control PID


WiFiServer server(5200);


void setup() {
  Serial.begin(115200);

  WiFi.beginAP(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  server.begin();


  // Initialize IR sensor pins
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_RIGHT, INPUT);

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_SPEED, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_SPEED, OUTPUT);

  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);


  attachInterrupt(digitalPinToInterrupt(ULTRA_SONIC_ECHO), UltrasonicSensor::ISR_Echo, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), lCount, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rCount, CHANGE);

  // Ensure motors start stopped
  stopMotors();
}

void loop() {
  /*WiFiClient client = server.available();

  //check if connected client has data to send
  if (client.connected() && client.available() >= 1) {
    String command = client.readStringUntil('\n');
    command.trim(); // Remove whitespace and newline characters
    handleCommand(command, client);
  } */
  
  Serial.println("Handling motor logic...");
  handleMotorLogic();
  handleUltrasonicSensor();

  float lDistance = (lTicks * wheelCircumference) / 8.0;
  float rDistance = (rTicks * wheelCircumference) / 8.0;
  float mDistance = (lDistance + rDistance) / 2.0;
 

 if (objectDetected) {
    setMotorSpeed(50, 50);  // Slow down instead of stopping
  }

  Serial.print("Left: ");
  Serial.print(lTicks);
  Serial.print(" Right: ");
  Serial.println(rTicks);

  Serial.print("Left Motor: "); Serial.print(analogRead(LEFT_MOTOR_SPEED));
  Serial.print(" Right Motor: "); Serial.println(analogRead(RIGHT_MOTOR_SPEED));

  Serial.print("Going: ");
  Serial.print(going);
  Serial.print(" Stopped: ");
  Serial.println(stopped);

}


// Function to handle ultrasonic sensor logic
void handleUltrasonicSensor() {
    unsigned long currentTime = millis();

    if (currentTime - lastUltrasonicTime >= ULTRASONIC_INTERVAL) {
        lastUltrasonicTime = currentTime;
        ultrasonic.trigger();
        unsigned long distance = ultrasonic.getDistance();

        if (distance > 0 && distance <= STOP_DISTANCE) {
            objectDetected = true;
            Serial.println("Obstacle detected! Slowing down...");
            double newSpeed = followPID.compute(distance, 15); // Target: 15cm
            setMotorSpeed(newSpeed, newSpeed);  // Reduce speed
        } else {
            objectDetected = false;
        }
    }
}

/*void handleUltrasonicSensor() {
    unsigned long currentTime = millis();

    if (currentTime - lastUltrasonicTime >= ULTRASONIC_INTERVAL) {
        lastUltrasonicTime = currentTime;
        ultrasonic.trigger();
        unsigned long distance = ultrasonic.getDistance();

        if (distance > 0 && distance <= STOP_DISTANCE) {
            objectDetected = true;

            // Compute new speed based on distance from object
            double newSpeed = followPID.compute(distance, 15); // Target is 15cm

            setMotorSpeed(newSpeed, newSpeed);
        } else {
            objectDetected = false;
        }
    }
}*/


// function to take a string sent from processing and make the buggy react
void handleCommand(String command, WiFiClient client) {

  if (command.equals("START")) {
    going = true;
    stopped = false;
    moveForward(150);  // Start moving
  }

    //Serial.println("Buggy starting...");
   
  
  else if (command.equals("STOP")) {
    going = false;
    stopped = true;
    stopMotors();
    server.print("DISTANCE TRAVELLED,");
    server.println(mDistance);
    total_Distance = 0;
    while(true){
      String command2;
      if (client.connected() && client.available()) {
        command2 = client.readStringUntil('\n');
      }
      if (command2.equals("START")){
        going = true;
        stopped = false;
        break;
      }
    }
    //Serial.println("Buggy stopping...");
  }
  else if (command.equals("SPEED1")){
    
  }
  else if (command.equals("SPEED2")){
    
  }
  else if (command.equals("SPEED3")){
    going = true;
    stopped = false;
    moveForward(150);  // Start moving
  }

}
