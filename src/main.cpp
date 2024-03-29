#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"
#include "normalizers.h"

Servo steering;

#define frontMax 200
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
int setAcceleration = 35; // m^2/s.

bool isReversing = false;
int finishedReversing = 0;

// value between 0, 1
float weight_sides, weight_frontSides;

void handleDcMotor()
{
  if (finishedReversing > 15)
  {
    isReversing = false;
    finishedReversing = 0;
  }
  if (isReversing)
  {
    if (distance[front] > 50)
    {
      finishedReversing++;
      return;
    }
    else
    {
      finishedReversing--;
      return;
    }
  }
  if ((distance[front] < 20) && (distance[frontLeft] < 20))
  {
    steering.write(50);
    targetSpeed = 75;
    reverse(setSpeed);
    digitalWrite(brakeLights, HIGH);
    isReversing = true;
    return;
  }
  else if ((distance[front] < 20) && (distance[frontRight] < 20))
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

  if (distance[front] > frontMax)
  {
    targetSpeed = 225;
  }
  else if (distance[front] > 125)
  {
    targetSpeed = logSpeedNormalizer(distance[front], 6, 1.245);
  }
  else
  {
    targetSpeed = eclipseSpeedNormalizer(distance[front], 1500, 350);
  }

  forward(targetSpeed);
}

void handleSteering()
{
  if ((millis() - lastSteeringChange) < 10 || isReversing)
  {
    return;
  }
  // resetMaxes();
  float differenceSide = (distance[left] - distance[right]) / maxSensorDistance;
  float differenceFrontSide = (distance[frontLeft] - distance[frontRight]) / maxSensorDistance;

  float differenceSum = differenceFrontSide * 0.7 + differenceSide * 0.3;
  targetAngle = (90.0 * (differenceFrontSide * -1 + 1.0));

  if (targetAngle > 140)
  {
    targetAngle = 140;
  }
  else if (targetAngle < 40)
  {
    targetAngle = 40;
  }
  if ((targetAngle > 90 && previousAngle < 90) || (targetAngle < 90 && previousAngle > 90))
  {
    steering.write(90);
  }

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

  if (targetSpeed - setSpeed < 0)
  {
    setSpeed = targetSpeed;
    lastSpeedChange = millis();
    return;
  }

  if (targetSpeed - setSpeed < setAcceleration)
  {
    setSpeed += setAcceleration / 100;
  }
  else
  {
    setSpeed = targetSpeed;
  }

  lastSpeedChange = millis();
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