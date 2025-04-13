#include "config.h"
#include "m.h"
#include "us.h"
#include "we.h"
#include "s.h"
#include <WiFiS3.h>

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

  pinMode(HALL_SENSOR_LEFT, INPUT);
  pinMode(HALL_SENSOR_RIGHT, INPUT);


  attachInterrupt(digitalPinToInterrupt(ULTRA_SONIC_ECHO), UltrasonicSensor::ISR_Echo, CHANGE);

  // Ensure motors start stopped
  stopMotors();
}

void loop() {
  WiFiClient client = server.available();

  //check if connected client has data to send
  if (client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim(); // Remove whitespace and newline characters
    handleCommand(command, client);
  }
  handleMotorLogic();
  handleUltrasonicSensor();
  rstate = digitalRead(HALL_SENSOR_RIGHT);
  lstate = digitalRead(HALL_SENSOR_LEFT);
  if (rstate == HIGH){
    revcountright++;
  }
  if (lstate == HIGH){
    revcountleft++;
  }
  total_Distance = (((revcountleft+revcountright)/2))*PI*5;


  if (objectDetected) {
    stopMotors();
    delay(300);
  }
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
      server.print("DETECTED,");
      server.println(distance);
      if(justChanged){
      Serial.println("Object detected! Stopping buggy.");
      }
    } else {
      objectDetected = false;
      if (justChanged){
      Serial.println("Path Clear! Starting Motors.");
      }
    }
  }
}

// function to take a string sent from processing and make the buggy react
void handleCommand(String command, WiFiClient client) {

  if (command.equals("START")) {
    going = true;
    stopped = false;
    //Serial.println("Buggy starting...");
  } 
  
  else if (command.equals("STOP")) {
    going = false;
    stopped = true;
    stopMotors();
    server.print("DISTANCE TRAVELLED,");
    server.println(total_Distance);
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

}
