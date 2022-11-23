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
int setAcceleration = 35; // m^2/s.

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
    targetSpeed = 150;
  }
  else if (distance[front] > 150)
  {
    targetSpeed = logSpeedNormalizer(distance[front], 6, 1.47);
  }
  else
  {
    targetSpeed = eclipseSpeedNormalizer(distance[front], 1500, 200);
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

  // if (differenceFrontSide > 0.25 && differenceFrontSide < 0.5)
  // {
  //   differenceFrontSide = 0.5;
  // }
  // else if (differenceFrontSide < -0.25 && differenceFrontSide > -0.5)
  // {
  //   differenceFrontSide = -0.5;
  // }
  differenceSide *= 0;
  targetAngle = (90.0 * (differenceFrontSide * -1 + 1.0));
  Serial.print(differenceFrontSide);
  Serial.println(" " + String(targetAngle));
  if (targetAngle > 135)
  {
    targetAngle = 135;
  }
  else if (targetAngle < 45)
  {
    targetAngle = 45;
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