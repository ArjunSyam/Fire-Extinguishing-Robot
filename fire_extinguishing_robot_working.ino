
#include <AFMotor.h>
#include <Servo.h>

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

const int maxSpeed = 100;    // Maximum speed for the motors
const int acceleration = 5;  // Acceleration rate for smooth speed changes
const int relay = 1;
const int flamePin1 = 9;  // Digital pins for flame sensors
const int flamePin2 = 11;
//const int flamePin3 = 10;

Servo myServo;      // Create a Servo object
int servoPin = 10;  // Pin for servo motor

void setup() {
  Serial.begin(9600);

  //moveForward();
  //delay(1000); // Add a delay if needed
  //stop();

  pinMode(flamePin1, INPUT_PULLUP);  // Set flame sensor pins as inputs with internal pull-up resistors
  pinMode(flamePin2, INPUT_PULLUP);
  pinMode(relay, OUTPUT);
    //pinMode(flamePin3, INPUT_PULLUP);

    motor1.setSpeed(maxSpeed);
  motor2.setSpeed(maxSpeed);
  motor3.setSpeed(maxSpeed);
  motor4.setSpeed(maxSpeed);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  moveForward();
  delay(1000);  // Add a delay if needed
  stop();

  myServo.attach(servoPin);  // Attach servo to pin 11
  myServo.write(90);         // Set initial servo position to 90 degrees
}

void loop() {
  if (!flameDetected()) {
    moveForward();
    delay(2000);

  } else {
    stop();
    delay(500);
    digitalWrite(relay,LOW);
    delay(50);
    swiveServo();
    delay(3000);
    digitalWrite(relay,HIGH);
  }
}

bool flameDetected() {
  // Check if any of the flame sensor pins are LOW (flame detected)
  Serial.println(digitalRead(flamePin1));
  Serial.println(digitalRead(flamePin2));
  //Serial.println(digitalRead(flamePin3));
  if (digitalRead(flamePin1) == LOW || digitalRead(flamePin2) == LOW /*|| digitalRead(flamePin3) == LOW*/) {
    return true;  // Flame detected
  } else {
    return false;  // No flame detected
  }
}

void moveForward() {
  Serial.println("Moving forward\n");
  accelerateMotors(motor1, motor2, motor3, motor4, maxSpeed);
}

void stop() {
  Serial.println("Stopped\n");
  decelerateMotors(motor1, motor2, motor3, motor4);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void accelerateMotors(AF_DCMotor& motor1, AF_DCMotor& motor2, AF_DCMotor& motor3, AF_DCMotor& motor4, int targetSpeed) {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  for (int speed = 0; speed <= targetSpeed; speed += acceleration) {
    motor1.setSpeed(speed);
    motor2.setSpeed(speed);
    motor3.setSpeed(speed);
    motor4.setSpeed(speed);
    delay(10);
  }
}

void decelerateMotors(AF_DCMotor& motor1, AF_DCMotor& motor2, AF_DCMotor& motor3, AF_DCMotor& motor4) {
  int currentSpeed = 100;
  while (currentSpeed > 0) {
    motor1.setSpeed(currentSpeed);
    motor2.setSpeed(currentSpeed);
    motor3.setSpeed(currentSpeed);
    motor4.setSpeed(currentSpeed);
    currentSpeed -= acceleration;
    delay(10);
  }
}

void swiveServo() {
  for (int angle = 90; angle >= 0; angle -= 5) {
    myServo.write(angle);
    delay(50);
  }
  for (int angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    delay(50);
  }
  for (int angle = 180; angle >= 90; angle -= 5) {
    myServo.write(angle);
    delay(50);
  }
}
