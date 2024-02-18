float Kp = 0.5, Ki = 0.5, Kd = 3;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

// sensors and sensor data
int irSensors[] = {A4, A3, A2, A1, A0}; // IR sensor pins
int irReadings[5];
int irAnalogData[5];

// left motor
int motorLForward = 4;
int motorLBackward = 5;
int motorLpwmPin = 10;

// right motor
int motorRForward = 6;
int motorRBackward = 7;
int motorRpwmPin = 9;

int initial_motor_speed = 200;
int turnspeed = 230;

void setup() {
  pinMode(motorLForward, OUTPUT);
  pinMode(motorLBackward, OUTPUT);
  pinMode(motorLpwmPin, OUTPUT);

  pinMode(motorRForward, OUTPUT);
  pinMode(motorRBackward, OUTPUT);
  pinMode(motorRpwmPin, OUTPUT);

  // sensor pins as INPUT
  for (int i = 0; i < 5; i++) {
    pinMode(irSensors[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  // Reading Sensor Values
  for (int i = 0; i < 5; i++) {
    irReadings[i] = digitalRead(irSensors[i]);
    irAnalogData[i] = analogRead(irSensors[i]);
  }

  // Line Following PID
  error = (irAnalogData[1] - irAnalogData[3]);
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;

  int leftMotorSpeed = initial_motor_speed - PID_value;
  int rightMotorSpeed = initial_motor_speed + PID_value;

  if (leftMotorSpeed > 255) {
    leftMotorSpeed = 255;
  }
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }
  if (rightMotorSpeed > 255) {
    rightMotorSpeed = 255;
  }
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0;
  }

  // Control motors
  analogWrite(motorLpwmPin, leftMotorSpeed);
  analogWrite(motorRpwmPin, rightMotorSpeed);

  digitalWrite(motorLForward, HIGH);
  digitalWrite(motorLBackward, LOW);

  digitalWrite(motorRForward, HIGH);
  digitalWrite(motorRBackward, LOW);
}
