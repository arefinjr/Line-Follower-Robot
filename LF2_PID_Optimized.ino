
/*
Optimized PID Code for LF-2 Robot (8-channel Analog Sensor)
Features:
- Anti-windup
- Filtered derivative
- Dynamic speed scaling
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// -------- Motor Driver Pins (TB6612FNG) --------
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10

// -------- Settings --------
bool isBlackLine = 1; // 1 = black line, 0 = white line
unsigned int numSensors = 8;

// PID Constants (Tune These)
float Kp = 0.04;
float Ki = 0.0005;
float Kd = 0.25;

// Speed Settings
int maxSpeed = 120;   // Max forward speed
int minSpeed = 30;    // Base speed
int currentSpeed = 30;

// Sensor Weights
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };

// Variables
int P, D, I, previousError, PIDvalue;
double error, filteredError;
int lsp, rsp;
int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
int activeSensors;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP); // Pushbutton
  pinMode(12, INPUT_PULLUP); // Pushbutton
  pinMode(13, OUTPUT);       // LED

  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); // Enable motor driver
}

void loop() {
  while (digitalRead(11)) {} // Wait for start button
  delay(500);
  calibrate();
  while (digitalRead(12)) {} // Wait for second button
  delay(500);

  while (1) {
    readLine();
    if (currentSpeed < maxSpeed) currentSpeed++;

    if (onLine) {
      lineFollow();
      digitalWrite(13, HIGH);
    } else {
      // Recovery mode: turn towards last error direction
      digitalWrite(13, LOW);
      if (error < 0) {
        motor1run(0);
        motor2run(minSpeed + 50);
      } else {
        motor1run(minSpeed + 50);
        motor2run(0);
      }
    }
  }
}

void lineFollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;

  // Low-pass filter on error to reduce noise
  filteredError = 0.7 * filteredError + 0.3 * error;

  // PID calculations
  P = filteredError;
  I += filteredError;
  I = constrain(I, -500, 500); // Anti-windup
  D = filteredError - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = filteredError;

  // Dynamic speed scaling for turns
  int turnFactor = abs(PIDvalue) / 3;
  int baseSpeed = maxSpeed - turnFactor;
  baseSpeed = constrain(baseSpeed, minSpeed, maxSpeed);

  lsp = baseSpeed - PIDvalue;
  rsp = baseSpeed + PIDvalue;

  lsp = constrain(lsp, -255, 255);
  rsp = constrain(rsp, -255, 255);

  motor1run(rsp);
  motor2run(lsp);
}

void calibrate() {
  for (int i = 0; i < 8; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) { // 3-second calibration
    motor1run(70);
    motor2run(-70);

    for (int i = 0; i < 8; i++) {
      int val = analogRead(i);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 8; i++) {
    if (isBlackLine)
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    else
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    if (sensorArray[i]) onLine = 1;
  }
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
