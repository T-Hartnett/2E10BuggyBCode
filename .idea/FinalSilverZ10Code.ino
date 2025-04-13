#include <WiFiS3.h>
#include <PID_v1.h>


//Pin number declarations
// Define IR sensor pins
#define IR_SENSOR_LEFT A5
#define IR_SENSOR_RIGHT A3

// Define motor control pins
#define LEFT_MOTOR_FORWARD 4
#define LEFT_MOTOR_BACKWARD 7
#define LEFT_MOTOR_SPEED 10
#define RIGHT_MOTOR_FORWARD 9
#define RIGHT_MOTOR_BACKWARD 1
#define RIGHT_MOTOR_SPEED 11

#define ULTRA_SONIC_TRIG 5
#define ULTRA_SONIC_ECHO 6

#define ENCODER_LEFT 2
#define ENCODER_RIGHT 3

//PID constants
double kpF = 5.7;
double kiF = 0.0005;
double kdF = 5.5;
double kpS = 0.6; 
double kiS = 0.8; 
double kdS = 0;

double inputS = 0;
double outputS = 0;
double setPointS = 0;
double inputF = 0;
double outputF = 0;
double setPointF = 0;

//used when in speed control mode
PID speedPID (&inputS, &outputS, &setPointS, kpS, kiS, kdS, DIRECT);


bool Following; // this bool switches the buggy in and out of follow mode
int speed; // PWM value 
int loopCount = 0;
const int THRESHOLD = 500;

//variables to calculate the current speed and new PID outputs
double previous_Time = 0, previous_distance = 0, prev_speed = 0;
unsigned long currentTime = 0, previousTime = 0;
unsigned long startTime = 0, endTime = 0;
int echoDuration = 0;
volatile bool isReading = false;
double elapsedTime = 0;
double error = 0;
double lastError = 0;
double input, output, setPoint, last_out =0;
double intError, devError;

bool isMoving = false; // Flag to indicate if the buggy is moving
volatile int wheelCountR = 0; // counts the number of highs from the wheel encoders
volatile int wheelCountL = 0; 
volatile float USdistance;
volatile bool objectDetected = false;
const unsigned long ULTRASONIC_INTERVAL = 100;  // Interval for ultrasonic readings (ms)
const int STOP_DISTANCE_F = 10;
const int STOP_DISTANCE_S = 20;  // Distance in cm at which the buggy should stop
const int MAX_RANGE = 40;
unsigned long lastUltrasonicTime = 0;

//WiFi set up
int status = WL_IDLE_STATUS;

char ssid[] = "BuggyTeamZ10";
char pass[] = "123456789";

WiFiServer server(5200);

void ISR_Echo();
void updateEncoderR();
void updateEncoderL();

void setup() {

  Serial.begin(115200);
  // connect to network
  status = WiFi.beginAP(ssid, pass);
  delay(1000);
  Serial.println("Connected to WiFi. My address:");
  IPAddress myAddress = WiFi.localIP();
  Serial.println(myAddress);
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

  pinMode(ENCODER_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT, INPUT_PULLUP);

  pinMode(ULTRA_SONIC_TRIG, OUTPUT);
  pinMode(ULTRA_SONIC_ECHO, INPUT_PULLUP);


  //set setpoint for follow mode to be 15 cm
  setPointF = 15;

  // set up interrupt pins for the wheel encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), updateEncoderL, CHANGE);
  // set up interupt for ultra-sonic Echo pin  
  attachInterrupt(digitalPinToInterrupt(ULTRA_SONIC_ECHO), ISR_Echo, CHANGE);

  //Set up speed PID with Library
  speedPID.SetSampleTime(1000);

  speedPID.SetOutputLimits(0, 255);

  speedPID.SetMode(AUTOMATIC);

  endTime = micros();
}


void loop() {

  WiFiClient client = server.available();

  //check if connected client has data to send
  if (client.connected() && client.available() >= 1) {
    String command = client.readStringUntil('\n');
    command.trim(); // Remove whitespace and newline characters
    handleCommand(command, client);
  } 
  
  if (isMoving) { // Only perform these operations if the buggy is moving
    
    int leftIR = analogRead(IR_SENSOR_LEFT);
    int rightIR = analogRead(IR_SENSOR_RIGHT);

    bool leftDetected = leftIR > THRESHOLD;
    bool rightDetected = rightIR > THRESHOLD;


    if (leftDetected && rightDetected) {
        forward(speed);
    } else if (leftDetected) {
        turnLeft(speed);
    } else if (rightDetected) {
        turnRight(speed);
    } else {
        stop();
    }
  

    
//every 200 loops check speed and update PID
    if (loopCount % 30 == 0) {
      double vel_speed = calculateSpeed();
      String speedMessage = "speed:" + String(vel_speed, 2); // Convert to string with 2 decimal places
      server.println(speedMessage); // Send the speed to the Processing application
    
    
      
      if (Following){
        
        handleUltraSonic();
        Serial.print("DISTANCE: ");
        Serial.println(USdistance);

        // if the object is out of range go to speed control settings
        if (USdistance > MAX_RANGE){
          inputS = vel_speed;
          setPointS = 20;
          speedPID.Compute();   // using PID calculate a new pwm value 
          speed = outputS; 
        }

      
        // otherwise if object is in range calculate a new pwm value using PID
        else if (USdistance > STOP_DISTANCE_F && USdistance < MAX_RANGE) {
          inputF = USdistance;
          outputF = computeFollowPID(inputF);  //find output using function PID 
          speed = outputF;

        }
      
        // if the object is less than 4cm away immediately stop
        else if (USdistance <= STOP_DISTANCE_F){
          stop();
        }

        Serial.println(speed);

      }

      // if not in following mode update speed based on current speed
      if (!Following) {
        handleUltraSonic();
        inputS = vel_speed;
        speedPID.Compute();   // using PID calculate a new pwm value 
        //Serial.print("OutputS: ");
        //Serial.println(outputS);
        speed = outputS; 
        Serial.println(speed);

        Serial.print("Detected?: ");
        Serial.println(objectDetected);

        if (objectDetected){
          speed = 0; 
        }
      }

      analogWrite(LEFT_MOTOR_SPEED, speed);
      analogWrite(RIGHT_MOTOR_SPEED, speed);

    } 
  }
  
  else { // if stopped delay a second between loops
    delay(100);
  }
    
  loopCount++;
}


void turnRight(int speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, speed * 1.2);
  //reverses the right wheel
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(RIGHT_MOTOR_SPEED, speed * 0.35);
}


void turnLeft(int speed) {
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(RIGHT_MOTOR_SPEED, speed * 1.2);
  // reverses the left wheel
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, speed * 0.35);
}


void stop() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, 0);
  analogWrite(RIGHT_MOTOR_SPEED, 0);
}


void forward(int speed) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  analogWrite(LEFT_MOTOR_SPEED, speed);
  analogWrite(RIGHT_MOTOR_SPEED, speed);
}


void updateEncoderR() {
    wheelCountR++;
    //Serial.print("Wheel Count - R: ");
    //Serial.print(wheelCountR);
}

void updateEncoderL() {
    wheelCountL++;
    //Serial.print(" - L: ");
    //Serial.println(wheelCountL);
}



// function to take a string sent from processing and make the buggy react
void handleCommand(String command, WiFiClient client) {
  Serial.println(command);

  if (command.equals("START")) {
    isMoving = true;
    Serial.print("Starting with speed: ");
    Serial.println(speed);
  } 
  
  else if (command.equals("STOP")) {
    isMoving = false;
    stop();
    Serial.println("Buggy stopping...");
    Serial.println(speed);
  } 
  
   else if (command.equals("Mode1")) {
    Following = true;
    Serial.print("Following: ");
    Serial.println(Following);
    Serial.println("Switched to Object Following Mode");

  } 
  
  else if (command.equals("Mode2")) {
    Following = false;
    Serial.print("Following: ");
    Serial.println(Following);
    Serial.println("Switched to Manual Speed Control Mode");
  } 

  else if (command.equals("Speed1")) {
    // set speed control's PID setpoint to the speed in cm/s sent by processing
    setPointS = 15;
    Serial.print("Speed set to ");
    Serial.println(setPointS);
  } 
  
  else if (command.equals("Speed2")) {
    // set speed control's PID setpoint to the speed in cm/s sent by processing
    setPointS = 20;
    Serial.print("Speed set to ");
    Serial.println(setPointS);
  }   
  
  else if (command.equals("Speed3")) {
    // set speed control's PID setpoint to the speed in cm/s sent by processing
    setPointS = 25;
    Serial.print("Speed set to ");
    Serial.println(setPointS);
  } 

  else {
    Serial.println("Unknown command.");
    Serial.println(command);
  }

}

double calculateSpeed() {
    unsigned long wheelCount = (wheelCountR + wheelCountL) / 2; 

    Serial.print("Wheel Count: "); Serial.println(wheelCount);

    double total_distance = (31.5 * 2 * 3.14 * (wheelCount / 8.0)) / 10.0;
    Serial.print("Total Distance: "); Serial.println(total_distance);
    server.print("Total Distance: "); server.println(total_distance);

    unsigned long current_Time = millis();
    double deltaT = (current_Time - previous_Time) / 1000.0;
    if (deltaT < 0.001) {
      deltaT = 0.001; // Avoid division by zero
    }
    Serial.print("Delta T: "); Serial.println(deltaT);

    double deltad = total_distance - previous_distance;
    if (deltad <= 0.1){
      deltad = 1.8; // Prevent divide-by-zero
    } 
    Serial.print("Delta D: "); Serial.println(deltad);

    double curr_speed = deltad / deltaT;
    Serial.print("Curr_Speed: "); Serial.println(curr_speed);

    previous_Time = current_Time;
    previous_distance = total_distance;
    prev_speed = curr_speed;

    return curr_speed;
}


// calculates PID equation for follow mode 
double computeFollowPID(double inputF){     
  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation
        
  error = inputF - setPointF;            
  intError = error * elapsedTime + intError;     // compute integral
  devError = (error - lastError)/elapsedTime;   // compute derivative

  double out = kpF * error + kiF * intError + kdF * devError;   //PID output   
              
  Serial.print("Error: ");
  Serial.println(error);

  /*server.print("Distance: ");
  server.println(inputF);
  server.print("Error: ");
  server.println(error);*/

  lastError = error;         
  previousTime = currentTime;   

  //set output limits 
  if(error <= 0){
    out = 0;
    } 
    else if (out < 50){
    out = 0;
  }
  else if (out > 130){
    out = 80;
  }
  
  

  Serial.print("Out: ");
  Serial.println(out);
  return out;   
}

void handleUltraSonic(){
  trigger();
  getDistance();

  USdistance = getDistance();

  if (USdistance >= 0 && USdistance <= STOP_DISTANCE_S) {
    objectDetected = true;
    server.print("DETECTED,");
    server.println(USdistance);
    Serial.print("Detected?: ");
    Serial.println(objectDetected);
  } else {
    objectDetected = false;
  }  
}



void trigger() {
  Serial.println("TRYING TO TRIGGER");
  //if (endTime + (20 * 1000) < micros()){
    Serial.println("TRIGGERING");
    //isReading = true;
    digitalWrite(ULTRA_SONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRA_SONIC_TRIG, HIGH);  
    delayMicroseconds(10);
    digitalWrite(ULTRA_SONIC_TRIG, LOW);
  //}
}

void ISR_Echo() {
  if (digitalRead(ULTRA_SONIC_ECHO) == HIGH) {
    startTime = micros();
  } else {
    endTime = micros();
    //isReading = false;
  }
}

float getDistance() {
  if (endTime > startTime){
    echoDuration = endTime - startTime;
    /*Serial.print("ECHO TIME: ");
    Serial.print(startTime);
    Serial.print(" ");
    Serial.print(endTime);
    Serial.print(" ");
    Serial.println(echoDuration);*/
  } 
  return echoDuration * 0.034 / 2; // Convert to cm
}