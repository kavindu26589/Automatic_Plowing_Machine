#define BLYNK_TEMPLATE_ID "TMPL6q_37b9nf"
#define BLYNK_TEMPLATE_NAME "ESP32Cam"
#define BLYNK_AUTH_TOKEN "ZQTeaC_UWGzLAG8KFbB_uDn3fh5EI2rd"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

// Blynk Auth Token and WiFi credentials
char auth[] = "ZQTeaC_UWGzLAG8KFbB_uDn3fh5EI2rd"; // Replace with your Blynk Auth Token
char ssid[] = "SSID";      // Replace with your WiFi SSID
char pass[] = "Password";  // Replace with your WiFi password

// Motor control pins for L298N
const int motor1Pin1 = 16; // Motor 1 IN1
const int motor1Pin2 = 17; // Motor 1 IN2
const int motor2Pin1 = 18; // Motor 2 IN3
const int motor2Pin2 = 21; // Motor 2 IN4

int xValue = 0;
int yValue = 0;
int motorSpeed = 255; // Default speed (0-255)

// Servo control pins
const int servo1Pin = 13;
const int servo2Pin = 14;
const int servoXPin = 27;
const int servoYPin = 26;

Servo servo1;
Servo servo2;
Servo servoX;
Servo servoY;

int posX = 90; // Initial position for X axis
int posY = 90; // Initial position for Y axis

// Joystick value range
const int joystickMin = -1023;
const int joystickMax = 1023;

const int motor1Enable = 32; // Motor 1 ENA
const int motor2Enable = 33; // Motor 2 ENB

// Separate motor control pin
const int MotorPin = 5; // Changed to avoid conflict with Camera Servo Y Pin
bool MotorPower = false;

bool isConnected = false; // Variable to track connection status
BlynkTimer timer; // Timer for checking connection status

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

  // Set separate motor pin as output
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

  // Initialize Blynk
  Blynk.begin(auth, ssid, pass);

  // Setup a timer to check connection status every second
  timer.setInterval(1000L, checkConnection);
}

void loop() {
  Blynk.run();
  timer.run();
}

// Function to control motors based on button presses
void controlMotors(int direction) {
  analogWrite(motor1Enable, motorSpeed);
  analogWrite(motor2Enable, motorSpeed);

  switch (direction) {
    case 1: // Forward
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      break;
    case 2: // Backward
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      break;
    case 3: // Left
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      break;
    case 4: // Right
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      break;
    default:
      // Stop all motors if no valid direction
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);
      break;
  }

  // Control separate motor based on MotorPower state
  if (MotorPower) {
    digitalWrite(MotorPin, HIGH);
  } else {
    digitalWrite(MotorPin, LOW);
  }
}

// Blynk vertical step slider for Servo 1
BLYNK_WRITE(V1) {
  int servo1Value = param.asInt();
  servo1.write(servo1Value);
  Serial.print("Servo 1: ");
  Serial.println(servo1Value);
}

// Blynk vertical step slider for Servo 2
BLYNK_WRITE(V2) {
  int servo2Value = param.asInt();
  servo2.write(servo2Value);
  Serial.print("Servo 2: ");
  Serial.println(servo2Value);
}

// Blynk slider for speed control
BLYNK_WRITE(V0) {
  motorSpeed = param.asInt();
  motorSpeed = constrain(motorSpeed, 0, 255); // Ensure the speed is within the PWM range
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);
}

// Blynk joystick control for X axis
BLYNK_WRITE(V7) {
  int xValue = param.asInt();
  posX = map(xValue, -1023, 1023, 60, 120); // Map -1023 to 1023 to 60 to 120 degrees
  servoX.write(posX);
  Serial.print("Cam_X: ");
  Serial.println(posX);
}

// Blynk joystick control for Y axis
BLYNK_WRITE(V8) {
  int yValue = param.asInt();
  posY = map(yValue, -1023, 1023, 60, 120); // Map -1023 to 1023 to 60 to 120 degrees
  servoY.write(posY);
  Serial.print("Cam_Y: ");
  Serial.println(posY);
}

// Blynk button for forward control
BLYNK_WRITE(V10) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(1); // Forward
  } else {
    controlMotors(0); // Stop
  }
}

// Blynk button for backward control
BLYNK_WRITE(V11) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(2); // Backward
  } else {
    controlMotors(0); // Stop
  }
}

// Blynk button for left control
BLYNK_WRITE(V12) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(3); // Left
  } else {
    controlMotors(0); // Stop
  }
}

// Blynk button for right control
BLYNK_WRITE(V13) {
  int buttonState = param.asInt();
  if (buttonState == HIGH) {
    controlMotors(4); // Right
  } else {
    controlMotors(0); // Stop
  }
}

// Blynk button for separate motor power control
BLYNK_WRITE(V6) {
  MotorPower = param.asInt();
  Serial.print("Motor Power: ");
  Serial.println(MotorPower ? "ON" : "OFF");
  controlMotors(0); // Stop motors when power is toggled
}

// Function to check connection status and update the dashboard LED
void checkConnection() {
  if (Blynk.connected()) {
    if (!isConnected) {
      Blynk.virtualWrite(V9, "0"); // Turn off LED first
      Blynk.setProperty(V9, "color", "#23C48E"); // Set color to green
      Blynk.virtualWrite(V9, "1"); // Turn on LED
      isConnected = true;
      Serial.println("Blynk connected");
    }
  } else {
    if (isConnected) {
      Blynk.virtualWrite(V9, "0"); // Turn off LED first
      isConnected = false;
      Serial.println("Blynk disconnected");
    }
    Blynk.virtualWrite(V9, millis() % 2000 < 1000 ? "1" : "0"); // Blink red LED
    Blynk.setProperty(V9, "color", "#D3435C"); // Set color to red
  }
}
