#define BLYNK_TEMPLATE_ID "TMPL6q_37b9nf"
#define BLYNK_TEMPLATE_NAME "ESP32Cam"
#define BLYNK_AUTH_TOKEN "ZQTeaC_UWGzLAG8KFbB_uDn3fh5EI2rd"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <WebServer.h>  // Include the WebServer library

// Blynk Auth Token and WiFi credentials
char auth[] = "ZQTeaC_UWGzLAG8KFbB_uDn3fh5EI2rd"; 
char ssid[] = "KAVINDU";      
char pass[] = "password";  

// Static IP configuration
IPAddress local_IP(192, 168, 1, 184);  // Set your desired static IP
IPAddress gateway(192, 168, 1, 1);     // Set your network gateway
IPAddress subnet(255, 255, 255, 0);    // Set your network subnet
IPAddress dns(192, 168, 1, 1);         // Set your DNS server

// Motor control pins for L298N
const int motor1Pin1 = 16; 
const int motor1Pin2 = 17; 
const int motor2Pin1 = 18; 
const int motor2Pin2 = 21; 

int xValue = 0;
int yValue = 0;
int motorSpeed = 255; 

// Servo control pins
const int servo1Pin = 13;
const int servo2Pin = 14;
const int servoXPin = 27;
const int servoYPin = 26;

Servo servo1;
Servo servo2;
Servo servoX;
Servo servoY;

int posX = 90; 
int posY = 90; 

const int motor1Enable = 32; 
const int motor2Enable = 33; 

// Separate motor control pin
const int MotorPin = 5; 
bool MotorPower = false;

bool isConnected = false; 
BlynkTimer timer; 

// Ultrasonic sensor pins
const int trigFront = 4;
const int echoFront = 2;

#define MAX_DISTANCE 200 
#define SAFE_DISTANCE 20 

bool autoMode = false; 
bool goButtonPressed = false; // Variable to check if "Go" button was pressed

MPU6050 mpu6050(Wire);

// Robot dimensions
const int robotLength = 30; // in cm
const int robotWidth = 10;  // in cm

// Room dimensions from Blynk
int roomWidth = 400;  
int roomLength = 800;  
int widthTime;  
int lengthTime;  
int speed = 100; 

// Web Server on port 80
WebServer server(80);

// String to hold log data
String logData = "";

void setup() {
  // Debug console
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Set motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(motor1Enable, OUTPUT);
  pinMode(motor2Enable, OUTPUT);

  pinMode(MotorPin, OUTPUT);
  digitalWrite(MotorPin, LOW);

  // Initialize servos without moving them
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Set initial positions
  servo1.write(90);
  servo2.write(90);
  servoX.write(90);
  servoY.write(90);

  // Delay to stabilize the servos
  delay(1000);

  // Initialize WiFi with static IP
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("Static IP configuration failed.");
  }
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);

  // Setup a timer to check connection status every second
  timer.setInterval(1000L, checkConnection);

  // Initialize MPU6050
  Wire.begin(15, 19);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Set up the Web Server routes
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/clear", handleClearLog); // Route to clear the log
  server.begin();

  // Calculate times for movement
  calculateMovementTimes();
}

void loop() {
  Blynk.run();
  timer.run();
  server.handleClient();

  // Only operate in auto mode if the mode is set to auto and "Go" button is pressed
  if (autoMode && goButtonPressed) {
    performRoomCoverageWithObstacleAvoidance();
  }
}

// Function to calculate movement times based on room dimensions
void calculateMovementTimes() {
  widthTime = (roomWidth / speed) * 1000;  
  lengthTime = (roomLength / speed) * 1000; 
}

// Blynk input for Room Width
BLYNK_WRITE(V20) {
  roomWidth = param.asInt();
  calculateMovementTimes();
  Serial.print("Room Width: ");
  Serial.println(roomWidth);
  logData += "Room Width set to: " + String(roomWidth) + " cm\n";
}

// Blynk input for Room Length
BLYNK_WRITE(V21) {
  roomLength = param.asInt();
  calculateMovementTimes();
  Serial.print("Room Length: ");
  Serial.println(roomLength);
  logData += "Room Length set to: " + String(roomLength) + " cm\n";
}

// Blynk input for Go Button
BLYNK_WRITE(V22) {
  int goState = param.asInt();
  if (goState == 1) {
    goButtonPressed = true;
    Serial.println("Go button pressed. Starting auto mode.");
    logData += "Go button pressed. Starting auto mode.\n";
  } else {
    goButtonPressed = false;
    Serial.println("Go button released. Auto mode not running.");
    logData += "Go button released. Auto mode not running.\n";
  }
}

// Function to control motors based on direction
void controlMotors(int direction) {
  analogWrite(motor1Enable, motorSpeed);
  analogWrite(motor2Enable, motorSpeed);

  switch (direction) {
    case 1: 
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      logData += "Moving Forward\n";
      break;
    case 2: 
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      logData += "Moving Backward\n";
      break;
    case 3: 
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      logData += "Turning Left\n";
      break;
    case 4: 
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      logData += "Turning Right\n";
      break;
    default:
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      logData += "Stopping\n";
      break;
  }
}

// Function to get the distance from the ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  logData += "Distance: " + String(distance) + " cm\n";
  return distance;
}

// Function to control the car in automatic mode with obstacle avoidance
void performRoomCoverageWithObstacleAvoidance() {
  logData += "Starting Room Coverage\n";

  Serial.println("Moving forward across the width...");
  moveForward();
  delayUntilObstacleAvoided(widthTime);

  stopCar();
  delay(1000);
  Serial.println("Rotating clockwise 90 degrees...");
  rotateClockwise90Degrees();

  Serial.println("Moving forward across the length...");
  moveForward();
  delayUntilObstacleAvoided(lengthTime);

  stopCar();
  delay(1000);
  Serial.println("Rotating clockwise 90 degrees...");
  rotateClockwise90Degrees();

  Serial.println("Moving forward across the width...");
  moveForward();
  delayUntilObstacleAvoided(widthTime);

  stopCar();
  Serial.println("Stopped at the end point.");
  logData += "Completed Room Coverage\n";
}

// Function to move forward and stop if an obstacle is detected
void delayUntilObstacleAvoided(int moveTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < moveTime) {
    long frontDist = getDistance(trigFront, echoFront);
    if (frontDist <= SAFE_DISTANCE) {
      stopCar();
      delay(500); // Small delay to stabilize before obstacle avoidance
      Serial.println("Obstacle detected! Avoiding...");
      logData += "Obstacle detected at " + String(frontDist) + " cm. Avoiding...\n";
      avoidObstacle();
    } else {
      moveForward();
    }
    delay(50); // Adjust as needed for smoother operation
  }
}

// Function to avoid an obstacle
void avoidObstacle() {
  // Step 1: Stop the car
  stopCar();
  delay(1000);

  // Step 2: Rotate 90 degrees to the right to move perpendicular to the obstacle
  rotateClockwise90Degrees();
  delay(1000);

  // Step 3: Move forward by the robot's width + a buffer to clear the obstacle
  moveForward();
  delay((robotWidth + 10) / speed * 1000); // Move forward to clear the obstacle

  // Step 4: Rotate 90 degrees to the left to realign with the original path
  rotateCounterClockwise90Degrees();
  delay(1000);

  // Step 5: Move forward by the robot's length + a buffer to ensure the robot is past the obstacle
  moveForward();
  delay((robotLength + 10) / speed * 1000);

  // Step 6: Rotate 90 degrees to the left to return to the original heading
  rotateCounterClockwise90Degrees();
  delay(1000);

  // Step 7: Move forward to continue the original path
  moveForward();
  logData += "Obstacle avoided successfully.\n";
}

// Movement functions (forward, stop, rotate)
void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void stopCar() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void rotateClockwise90Degrees() {
  float targetAngle = 90.0;
  float currentAngle = 0.0;
  
  mpu6050.update();
  float initialAngle = mpu6050.getAngleZ();
  
  while (currentAngle < targetAngle) {
    mpu6050.update();
    float deltaAngle = mpu6050.getGyroZ() * 0.01; 
    currentAngle += deltaAngle;

    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);

    delay(10); 
  }

  stopCar();
}

void rotateCounterClockwise90Degrees() {
  float targetAngle = -90.0;
  float currentAngle = 0.0;
  
  mpu6050.update();
  float initialAngle = mpu6050.getAngleZ();
  
  while (currentAngle > targetAngle) {
    mpu6050.update();
    float deltaAngle = mpu6050.getGyroZ() * 0.01; 
    currentAngle += deltaAngle;

    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);

    delay(10); 
  }

  stopCar();
}

// Blynk vertical step slider for Servo 1
BLYNK_WRITE(V1) {
  int servo1Value = param.asInt();
  servo1.write(servo1Value);
  Serial.print("Servo 1: ");
  Serial.println(servo1Value);
  logData += "Servo 1 set to: " + String(servo1Value) + "\n";
}

// Blynk vertical step slider for Servo 2
BLYNK_WRITE(V2) {
  int servo2Value = param.asInt();
  servo2.write(servo2Value);
  Serial.print("Servo 2: ");
  Serial.println(servo2Value);
  logData += "Servo 2 set to: " + String(servo2Value) + "\n";
}

// Blynk slider for speed control
BLYNK_WRITE(V0) {
  motorSpeed = param.asInt();
  motorSpeed = constrain(motorSpeed, 0, 255); 
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);
  logData += "Motor Speed set to: " + String(motorSpeed) + "\n";
}

// Blynk joystick control for X axis
BLYNK_WRITE(V7) {
  xValue = param.asInt();
  posX = map(xValue, -1023, 1023, 60, 120); 
  servoX.write(posX);
  Serial.print("Cam_X: ");
  Serial.println(posX);
  logData += "Cam_X set to: " + String(posX) + "\n";
}

// Blynk joystick control for Y axis
BLYNK_WRITE(V8) {
  yValue = param.asInt();
  posY = map(yValue, -1023, 1023, 60, 120); 
  servoY.write(posY);
  Serial.print("Cam_Y: ");
  Serial.println(posY);
  logData += "Cam_Y set to: " + String(posY) + "\n";
}

// Blynk button for forward control
BLYNK_WRITE(V10) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(1); 
  } else {
    controlMotors(0); 
  }
}

// Blynk button for backward control
BLYNK_WRITE(V11) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(2); 
  } else {
    controlMotors(0); 
  }
}

// Blynk button for left control
BLYNK_WRITE(V12) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(3); 
  } else {
    controlMotors(0); 
  }
}

// Blynk button for right control
BLYNK_WRITE(V13) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(4); 
  } else {
    controlMotors(0); 
  }
}

// Blynk button for separate motor power control
BLYNK_WRITE(V6) {
  MotorPower = param.asInt();
  Serial.print("Motor Power: ");
  Serial.println(MotorPower ? "ON" : "OFF");
  logData += "Motor Power: " + String(MotorPower ? "ON" : "OFF") + "\n";
  if (MotorPower) {
    digitalWrite(MotorPin, HIGH);
  } else {
    digitalWrite(MotorPin, LOW);
  }
}

// Blynk switch for control mode
BLYNK_WRITE(V5) {
  int mode = param.asInt();
  autoMode = (mode == 1); 
  Serial.print("Control Mode: ");
  Serial.println(autoMode ? "Automatic" : "Manual");
  logData += "Control Mode set to: " + String(autoMode ? "Automatic" : "Manual") + "\n";
  if (autoMode) {
    Serial.println("Auto mode activated.");
  } else {
    Serial.println("Manual mode activated.");
  }
}

// Function to check connection status and update the dashboard LED
void checkConnection() {
  if (Blynk.connected()) {
    if (!isConnected) {
      Blynk.virtualWrite(V9, "0"); 
      Blynk.setProperty(V9, "color", "#23C48E"); 
      Blynk.virtualWrite(V9, "1"); 
      isConnected = true;
      Serial.println("Blynk connected");
      logData += "Blynk connected\n";
    }
  } else {
    if (isConnected) {
      Blynk.virtualWrite(V9, "0"); 
      isConnected = false;
      Serial.println("Blynk disconnected");
      logData += "Blynk disconnected\n";
    }
    Blynk.virtualWrite(V9, millis() % 2000 < 1000 ? "1" : "0"); 
    Blynk.setProperty(V9, "color", "#D3435C"); 
  }
}

// Web server handlers
void handleRoot() {
  server.send(200, "text/html", "<html><head><meta http-equiv='refresh' content='1;url=/log'></head><body><h1>Automatic Plowing Machine Log Server</h1><p>Redirecting to log page...</p></body></html>");
}

void handleLog() {
  String html = "<html><head>";
  html += "<title>Automatic Plowing Machine Log</title>";
  html += "<style>body { font-family: Arial, sans-serif; margin: 20px; } #log { height: 90vh; overflow-y: scroll; border: 1px solid #ccc; padding: 10px; white-space: pre-wrap; }</style>";
  html += "<script>function scrollToBottom() { var logDiv = document.getElementById('log'); logDiv.scrollTop = logDiv.scrollHeight; }";
  html += "setInterval(() => { window.location.reload(); }, 1000);"; // Refresh every second
  html += "</script></head><body>";
  html += "<h1>Automatic Plowing Machine Log</h1>";
  html += "<div id='log'>" + logData + "</div>";
  html += "<button onclick='window.location.href=\"/clear\"'>Clear Log</button>"; // Add Clear Log button
  html += "<script>scrollToBottom();</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleClearLog() {
  logData = ""; // Clear the log data
  server.send(200, "text/html", "<html><body><h1>Log cleared!</h1><br><a href='/log'>Back to Log</a></body></html>");
}
