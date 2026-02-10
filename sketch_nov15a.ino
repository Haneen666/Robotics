#include <Servo.h>  // Include the Servo library

// Pin Definitions
#define IN1 3      // Motor 1 control pin 1 (L293D)
#define IN2 4      // Motor 1 control pin 2 (L293D)
#define IN3 5      // Motor 2 control pin 1 (L293D)
#define IN4 6      // Motor 2 control pin 2 (L293D)
#define ENA 9      // Motor 1 speed control (L293D)
#define ENB 10     // Motor 2 speed control (L293D)

#define LEFT_SENSOR_PIN 2   // Left TCRT5000 sensor
#define RIGHT_SENSOR_PIN 4  // Right TCRT5000 sensor
#define TRIG_PIN 7          // Ultrasonic sensor trig pin
#define ECHO_PIN 8          // Ultrasonic sensor echo pin

#define SERVO_PIN 11        // Pin to control the Servo

long duration;              // Used for ultrasonic sensor calculation
int distance;               // Distance in cm
Servo myServo;             // Create a Servo object

void setup() {
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor pins setup
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  // Ultrasonic sensor pins setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize servo
  myServo.attach(SERVO_PIN);  // Attach the servo to the defined pin
  myServo.write(90);          // Set the initial servo position (90 degrees)

  // Start motors at full speed
  analogWrite(ENA, 255);  // Full speed for motor 1
  analogWrite(ENB, 255);  // Full speed for motor 2
}

void loop() {
  // Read the values from the TCRT5000 sensors (0 or 1)
  int leftSensorValue = digitalRead(LEFT_SENSOR_PIN);
  int rightSensorValue = digitalRead(RIGHT_SENSOR_PIN);

  // Read distance from ultrasonic sensor
  distance = getDistance();

  // Check for obstacles (distance < 10 cm)
  if (distance < 10) {
    stopMotors();
    delay(500);
    turnRight();  // Turn right if obstacle is detected
    delay(1000);
  } 
  // If line-following logic
  else if (leftSensorValue == LOW && rightSensorValue == LOW) {
    moveForward();  // Move forward when both sensors see the line
  } 
  else if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    turnLeft();  // Turn left when the left sensor is off the line
  } 
  else if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    turnRight();  // Turn right when the right sensor is off the line
  }
  else {
    moveForward();  // If both sensors see the line, move forward
  }
}

// Function to move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  myServo.write(90);  // Keep the servo at the middle (90 degrees) to go straight
}

// Function to turn left
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  myServo.write(45);  // Turn the servo to 45 degrees for a left turn
}

// Function to turn right
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  myServo.write(135);  // Turn the servo to 135 degrees for a right turn
}

// Function to stop motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);  // Stop motor 1
  analogWrite(ENB, 0);  // Stop motor 2
  myServo.write(90);    // Reset servo to middle position
}

// Function to get distance from ultrasonic sensor
long getDistance() {
  // Send a pulse to trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the pulse duration from echo pin
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in cm
  long distanceCm = (duration / 2) / 29.1;  // Speed of sound in air is 343 m/s or 29.1 us/cm
  
  return distanceCm;
}
