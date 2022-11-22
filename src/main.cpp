#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"
#include "normalizers.h"

Servo steering;

#define frontMax 175
#define maxSpeed 250

#define sideMax 300
#define frontSideMax 450

#define brakeLights 9

int sensorToPrint = 0;

long lastSteeringChange = 0;
long lastDcChange = 0;
long lastSpeedChange = 0;
long debugChange = 0;

int previousAngle = 90;
int targetAngle = 90;

int reverseDistance = 10;

// only use this to SET the speed
int setSpeed = 0;
// use this when you want to change the target speed
int targetSpeed = 0;
int setAcceleration = 75; // m^2/s.

bool isReversing = false;

// value between 0, 1
float weight_sides, weight_frontSides;

void handleDcMotor()
{
  if (isReversing)
  {
    if (distance[front] > 35)
    {
      isReversing = false;
      return;
    }
    else
    {
      return;
    }
  }
  if ((distance[front] < 20) && (distance[frontLeft] < 25))
  {
    steering.write(50);
    targetSpeed = 75;
    reverse(setSpeed);
    digitalWrite(brakeLights, HIGH);
    isReversing = true;
    return;
  }
  else if ((distance[front] < 20) && (distance[frontRight] < 25))
  {
    steering.write(130);
    targetSpeed = 75;
    reverse(setSpeed);
    digitalWrite(brakeLights, HIGH);
    isReversing = true;
    return;
  }
  else
  {
    digitalWrite(brakeLights, LOW);
  }

  float steeringAngleFrac = 1.0 - (float(abs(targetAngle - 90)) / 90.0);
  if (distance[front] > frontMax)
  {
    targetSpeed = steeringAngleFrac < 0.8 ? 250.0 * steeringAngleFrac : 250;
    Serial.println(String(targetSpeed) + " " + String(setSpeed));
  }
  else
  {
    targetSpeed = float(speedNormalizer(distance[front], frontMax)) * steeringAngleFrac;
  }
  forward(setSpeed);
}

void handleSteering()
{
  if ((millis() - lastSteeringChange) < 10 || isReversing)
  {
    return;
  }
  // resetMaxes();
  float differenceSide = (distance[left] - distance[right]) / (distance[left] + distance[right]);
  float differenceFrontSide = (distance[frontLeft] - distance[frontRight]) / (distance[frontLeft] + distance[frontRight]);

  differenceFrontSide *= 0.5;
  differenceSide *= 0.5;
  targetAngle = angleNormalizer(differenceFrontSide + differenceSide);
  if ((targetAngle > 90 && previousAngle < 90) || (targetAngle < 90 && previousAngle > 90))
  {
    steering.write(90);
  }
  Serial.println(String(differenceFrontSide) + " " + String(differenceSide) + " - " + String(targetAngle));

  steering.write(targetAngle);
  previousAngle = targetAngle;
  lastSteeringChange = millis();
}

void handleAcceleration()
{
  if (setSpeed == targetSpeed || (millis() - lastSpeedChange < 10))
  {
    return;
  }

  if (targetSpeed < 80)
  {
    setSpeed = targetSpeed;
    return;
  }

  if (abs(targetSpeed - setSpeed) < setAcceleration)
  {
    setSpeed += setAcceleration / 100;
  }
  else
  {
    setSpeed = targetSpeed;
  }
}

void setup()
{
  Serial.begin(9600);
  steering.attach(7);

  initSensors();

  pinMode(dcInput1, OUTPUT);
  pinMode(dcInput2, OUTPUT);
  pinMode(brakeLights, OUTPUT);
}

void loop()
{
  // handleSerial();

  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();
    lastPollMillis = millis();
  }

  handleSteering();
  handleAcceleration();
  handleDcMotor();
}