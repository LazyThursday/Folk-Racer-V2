#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"

Servo steering;

#define frontMax 170
#define maxSpeed 250

#define sideMax 75
#define frontSideMax 150

int sensorToPrint = 0;

long lastSteeringChange = 0;
long lastDcChange = 0;
long lastSpeedChange = 0;

int previousAngle = 90;
int tempAngle = 90;

int reverseDistance = 10;

// only use this to SET the speed
int setSpeed = 0;
// use this when you want to change the target speed
int targetSpeed = 0;
int setAcceleration = 100; // m^2/s.

bool motorsOn = true;

void handleSerial()
{
  String input;
  if (Serial.available() > 0)
  {
    input = Serial.readStringUntil('\n');
  }
  else
  {
    return;
  }

  String command = input.substring(0, 1);

  if (command == "a")
  {
    int angle = input.substring(1, 3).toInt();

    if (angle >= 0 && angle <= 180)
    {
      steering.write(angle);
      Serial.println("Success!");
    }
    else
    {
      Serial.println("Invalid angle");
    }
  }
  else if (command == "f")
  {
    int speed = input.substring(1, 4).toInt();

    forward(speed);

    Serial.println("speed Set to" + String(speed));
  }
  else if (command == "r")
  {
    int speed = input.substring(1, 4).toInt();

    reverse(speed);

    Serial.println("speed Set to" + String(speed));
  }
  else if (command == "s")
  {
    stopMotor();
    Serial.println("Motor stopped");
  }
  else if (command == "u")
  {
    sensorToPrint = input.substring(1).toInt();

    Serial.println("\n\n Showing sensor " + String(sensorToPrint) + "\n\n");
    delay(1500);
  }
  else if ("k")
  {
    if (!motorsOn)
    {
      delay(3000);
    }
    motorsOn = !motorsOn;
    stopMotor();
  }

  return;
}

int speedNormalizer(int x, int a, int b)
{
  return (a * sqrt(x * (2 * b - x)) / b);
}

void handleDcMotor()
{
  if ((distance[front] < 20) && (distance[frontLeft] < 20))
  {
    steering.write(60);
    targetSpeed = 75;
    reverse(setSpeed);
    return;
  }
  else if ((distance[front] < 20) && (distance[frontRight] < 20))
  {
    steering.write(130);
    targetSpeed = 75;
    reverse(setSpeed);
    return;
  }

  if (distance[front] > frontMax)
  {
    targetSpeed = 250 * max(1 - (abs(tempAngle - 90)), 0.7);
  }
  else
  {
    targetSpeed = speedNormalizer(distance[front], frontMax, 100);
  }
  forward(setSpeed);
}

void resetMaxes()
{
  if (distance[left] > sideMax)
  {
    distance[left] = sideMax;
  }
  if (distance[right] > sideMax)
  {
    distance[right] = sideMax;
  }
  if (distance[frontLeft] > frontSideMax)
  {
    distance[frontLeft] = frontSideMax;
  }
  if (distance[frontRight] > frontSideMax)
  {
    distance[frontRight] = frontSideMax;
  }
}

void handleSteering()
{
  if ((millis() - lastSteeringChange) < 50)
  {
    return;
  }
  resetMaxes();
  tempAngle = 90 - ((distance[left] - distance[right]) / 1.95) - ((distance[frontLeft] - distance[frontRight]) / 1.95);
  // Serial.println(String(tempAngle) + " - " + String(distance[4]) + "cm");
  if (tempAngle > 130)
  {
    tempAngle = 130;
  }
  else if (tempAngle < 50)
  {
    tempAngle = 50;
  }
  if ((tempAngle > 90 && previousAngle < 90) || (tempAngle < 90 && previousAngle > 90))
  {
    steering.write(90);
  }
  steering.write(tempAngle);
  previousAngle = tempAngle;
  // Serial.println(String(distance[left]) + " - " + String(distance[frontLeft]) + " - " + String(distance[front]) + " - " + String(distance[frontRight]) + " - " + String(distance[right]));
  Serial.println(String(distance[frontLeft]) + " - " + String(distance[frontRight]) + " | " + String(distance[left]) + " - " + String(distance[right]) + " | " + tempAngle);
  lastSteeringChange = millis();
}

void handleAcceleration()
{
  if (setSpeed == targetSpeed || (millis() - lastSpeedChange < 100))
  {
    return;
  }

  if (abs(targetSpeed - setSpeed) < setAcceleration)
  {
    setSpeed += setAcceleration / 10;
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
}

void loop()
{
  handleSerial();

  if (millis() - lastPollMillis >= pingDelay)
  {
    doMeasurement();

    lastPollMillis = millis();
  }
  if (motorsOn)
  {
    handleSteering();
    handleAcceleration();
    handleDcMotor();
  }
}