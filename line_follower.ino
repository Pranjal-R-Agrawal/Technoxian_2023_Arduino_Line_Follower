#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <SparkFun_TB6612.h>

// Enter Line Details
bool isBlackLine = true;
unsigned int lineThickness = 25;  //Enter line thickness in mm. Works best for thickness between 10 & 35
bool brakeEnabled = 0;

// Motor Configuration
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 210;
int currentSpeed = 100;

float Kp = 0.12;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;
int numValues = 15;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorReadings[7][15];
int index = 0;
bool filled = 0;
bool takeAverage = 1;
bool brakeFlag = 0;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  lineThickness = constrain(lineThickness, 10, 35);
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
      brakeFlag = 0;
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        if (brakeEnabled == 1 && brakeFlag == 0) {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }
        motor1.drive(-50);
        motor2.drive(160);
        brakeFlag = 1;
      } else {
        if (brakeEnabled == 1 && brakeFlag == 0) {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }
        motor1.drive(160);
        motor2.drive(-50);
        brakeFlag = 1;
      }
    }
  }
}

void linefollow() {
  error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 2 * sensorValue[5] - 3 * sensorValue[6]);

  if (lineThickness > 22) {
    error = error * -1;
  }
  if (isBlackLine) {
    error = error * -1;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000; i++) {
    motor1.drive(40);
    motor2.drive(-40);

    for (int i = 0; i < 7; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  motor1.drive(0);
  motor2.drive(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 7; i++) {
    if (filled) {
      sensorValue[i] = sensorValue[i] * numValues - sensorReadings[i][index];
      sensorReadings[i][index] = constrain(map(analogRead(i), minValues[i], maxValues[i], 0, 1000), 0, 1000);
      sensorValue[i] = (sensorValue[i] + sensorReadings[i][index]) / numValues;
    }
    else {
      sensorValue[i] = constrain(map(analogRead(i), minValues[i], maxValues[i], 0, 1000), 0, 1000);
      sensorReadings[i][index] = sensorValue[i];
    }
    if (sensorValue[i] > 500) onLine = 1;
  }
  filled = (filled || (index == (numValues - 1))) && takeAverage;
  index = (index + 1) % (numValues - 1);
}