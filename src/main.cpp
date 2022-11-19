#include <Arduino.h>
#include <Servo.h>
#include "sensors.h"
#include "dcMotor.h"

Servo steering;

#define maxDetectionDistance 130
#define maxSpeed 250

int sensorToPrint = 0;

long lastSteeringChange = 0;
long lastDcChange = 0;
long lastSpeedChange = 0;

int previousAngle = 90;
int tempAngle = 90;

int reverseDistance = 20;

// only use this to SET the speed
int setSpeed = 0;
// use this when you want to change the target speed
int targetSpeed = 0;
int setAcceleration = 50; // m^2/s.

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

int speedNormalizer(int x, int w)
{
  return w * (log10(x + 20) / 0.07918124604);
  // return x / 2;
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

  if (distance[front] > 400)
  {
    distance[front] = 400;
  }

  if (distance[front] > 200)
  {
    targetSpeed = speedNormalizer(distance[front], 7);
  }
  else
  {
    targetSpeed = speedNormalizer(distance[front], 4);
  }
  forward(setSpeed);
}

void resetMaxes()
{
  if (distance[left] > 150)
  {
    distance[left] = 150;
  }
  if (distance[right] > 150)
  {
    distance[right] = 150;
  }
  if (distance[frontLeft] > 300)
  {
    distance[frontLeft] = 300;
  }
  if (distance[frontRight] > 300)
  {
    distance[frontRight] = 300;
  }
}

void handleSteering()
{
  if ((millis() - lastSteeringChange) < 10)
  {
    return;
  }
  resetMaxes();
  tempAngle = 90 - ((distance[left] - distance[right]) / 5) + ((distance[frontRight] - distance[frontLeft]) / 5);
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
  Serial.println(tempAngle);
  lastSteeringChange = millis();
}

void handleAcceleration()
{
  if (setSpeed == targetSpeed || (millis() - lastSpeedChange < 1000))
  {
    return;
  }

  if (abs(targetSpeed - setSpeed) < setAcceleration)
  {
    setSpeed += setAcceleration;
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